#!/usr/bin/env python3
"""Controle da cabeça InMoov (ESP32 + PCA9685) por serial e MediaPipe.

Permite controlar servos individualmente, aplicar expressões predefinidas e
espelhar expressões faciais capturadas pela webcam.

Dependências:
    pip install pyserial opencv-python mediapipe

Também é necessário colocar o arquivo face_landmarker.task na mesma pasta
 deste script. O protocolo serial esperado pelo ESP32 é uma linha com 17
valores separados por vírgulas (canais 0-15 no PCA9685 e canal 16 = rotação da
cabeça, ligado direto ao GPIO 13 do ESP32) ou o comando OFF.
"""

import json
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk

import cv2
import mediapipe as mp
import serial
import serial.tools.list_ports
from mediapipe.tasks.python import vision


# ===== Servos: nome, limites (ticks), inversão e canais de ajuste manual =====
SERVOS = [
    {"nome": "Maxilar",             "min": 150, "max": 321, "inv": False},
    {"nome": "Sobrancelha Esq.",    "min": 191, "max": 299, "inv": True},
    {"nome": "Testa Esq. (livre)",  "min": 150, "max": 600, "inv": False, "livre": True},
    {"nome": "Pálpebra Dir. Baixo", "min": 225, "max": 308, "inv": True},
    {"nome": "Pálpebra Dir. Cima",  "min": 210, "max": 327, "inv": False},
    {"nome": "Sobrancelha Dir.",    "min": 226, "max": 373, "inv": False},
    {"nome": "Testa Dir. (livre)",  "min": 150, "max": 600, "inv": False, "livre": True},
    {"nome": "Olho Dir. E/D",       "min": 251, "max": 306, "inv": True},
    {"nome": "Olho Dir. U/D",       "min": 228, "max": 355, "inv": False},
    {"nome": "Bochecha Dir.",       "min": 250, "max": 350, "inv": False},
    {"nome": "Pálpebra Esq. Cima",  "min": 250, "max": 310, "inv": True},
    {"nome": "Pálpebra Esq. Baixo", "min": 271, "max": 340, "inv": False},
    {"nome": "Olho Esq. E/D",       "min": 263, "max": 312, "inv": True},
    {"nome": "Olho Esq. U/D",       "min": 230, "max": 312, "inv": True},
    {"nome": "Bochecha Esq.",       "min": 250, "max": 350, "inv": True},
    {"nome": "Nariz",               "min": 271, "max": 327, "inv": False},
    # Canal 16: direto no GPIO 13 do ESP32. Limites PROVISÓRIOS (1,0–2,0 ms);
    # ajuste conforme o curso mecânico real do pescoço (e o do firmware).
    {"nome": "Rotação Cabeça",     "min": 200, "max": 400, "inv": False},
]

NUM_SERVOS = len(SERVOS)
CANAL_ROTACAO = 16
LINHAS_SERVOS = 9  # sliders por coluna (17 canais -> 9 + 8)
PALP_CIMA = (4, 10)
PALP_BAIXO = (3, 11)
OLHO_ED = (7, 12)
OLHO_UD = (8, 13)
BOCHECHAS = (9, 14)

DIRETORIO_SCRIPT = Path(__file__).resolve().parent
ARQUIVO_PRESETS = DIRETORIO_SCRIPT / "presets_personalizados.json"
ARQUIVO_MODELO = DIRETORIO_SCRIPT / "face_landmarker.task"
BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
BAUD_PADRAO = 115200
BOTOES_POR_LINHA = 6
INTERVALO_CAMERA_MS = 33  # aproximadamente 30 FPS
INTERVALO_FILA_MS = 50


def limitar(valor, minimo, maximo):
    """Limita um valor a um intervalo inclusivo."""
    return max(minimo, min(maximo, valor))


def exp(
    maxilar=0,
    sobr_e=50,
    sobr_d=50,
    palp_cima=80,
    palp_baixo=80,
    olho_ed=50,
    olho_ud=50,
    bochecha=0,
    nariz=0,
):
    """Monta um preset facial em porcentagens, indexado pelo canal do servo."""
    dados = {0: maxilar, 1: sobr_e, 5: sobr_d, 15: nariz}
    for canal in PALP_CIMA:
        dados[canal] = palp_cima
    for canal in PALP_BAIXO:
        dados[canal] = palp_baixo
    for canal in OLHO_ED:
        dados[canal] = olho_ed
    for canal in OLHO_UD:
        dados[canal] = olho_ud
    for canal in BOCHECHAS:
        dados[canal] = bochecha
    return dados


PRESETS_PADRAO = {
    "Neutro": exp(),
    "Feliz": exp(maxilar=25, sobr_e=65, sobr_d=65, palp_cima=85,
                  palp_baixo=55, bochecha=100, nariz=20),
    "Sorrindo": exp(sobr_e=55, sobr_d=55, palp_baixo=70,
                    bochecha=100, nariz=10),
    "Triste": exp(maxilar=5, sobr_e=30, sobr_d=30, palp_cima=50,
                  palp_baixo=85, olho_ud=25),
    "Bravo": exp(sobr_e=0, sobr_d=0, palp_cima=70, palp_baixo=40, nariz=80),
    "Surpreso": exp(maxilar=80, sobr_e=100, sobr_d=100, palp_cima=100,
                    palp_baixo=100, nariz=30),
    "Desconfiado": exp(sobr_e=100, sobr_d=20, palp_cima=55,
                       palp_baixo=60, olho_ed=70),
    "Sonolento": exp(maxilar=10, sobr_e=40, sobr_d=40, palp_cima=25,
                     palp_baixo=40, olho_ud=35),
    "Dormindo": exp(maxilar=10, sobr_e=40, sobr_d=40,
                    palp_cima=0, palp_baixo=0),
    "Olhar Esq.": {7: 0, 12: 0},
    "Olhar Dir.": {7: 100, 12: 100},
}


def pct_para_tick(canal, percentual):
    servo = SERVOS[canal]
    percentual = limitar(float(percentual), 0, 100)
    if servo["inv"]:
        percentual = 100 - percentual
    return round(
        servo["min"] + (servo["max"] - servo["min"]) * percentual / 100
    )


def tick_para_pct(canal, tick):
    servo = SERVOS[canal]
    intervalo = servo["max"] - servo["min"]
    if intervalo == 0:
        return 0
    percentual = (float(tick) - servo["min"]) / intervalo * 100
    if servo["inv"]:
        percentual = 100 - percentual
    return round(limitar(percentual, 0, 100))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Controle InMoov + Espelhamento Facial")
        self.resizable(False, False)

        self.ser = None
        self._lock_serial = threading.Lock()
        self.leitor_ativo = False
        self.fila_rx = queue.Queue()
        self._thread_rx = None
        self._fechando = False

        self.sliders = []
        self.entradas = []
        self.vars_entrada = []
        self._atualizando = False
        self._anim_id = None

        self.cap = None
        self.detector = None
        self.espelhar_rostos = tk.BooleanVar(value=False)
        self._ultimo_erro_camera = None
        # Estado do filtro passa-baixas EMA, inicializado ao detectar o primeiro rosto.
        self._filtro_facial = {}
        self.var_filtro_alpha = tk.DoubleVar(value=0.30)

        self.presets_custom = self._carregar_custom()
        self.frame_botoes = None
        self.presets = {}

        self._montar_topo()
        self._montar_presets()
        self._montar_servos()
        self._montar_rodape()
        self.atualizar_portas()
        self._desenhar_botoes_presets()

        self.after(INTERVALO_FILA_MS, self._processar_fila)
        self.after(100, self._processar_camera)
        self.protocol("WM_DELETE_WINDOW", self._fechar)

    # ----------------------------- Interface -----------------------------
    def _montar_topo(self):
        quadro = ttk.LabelFrame(self, text="Conexão serial & câmera")
        quadro.pack(fill="x", padx=8, pady=6)

        ttk.Label(quadro, text="Porta:").grid(row=0, column=0, padx=4, pady=4)
        self.cb_porta = ttk.Combobox(quadro, width=15, state="readonly")
        self.cb_porta.grid(row=0, column=1, padx=4)
        ttk.Button(quadro, text="Atualizar", command=self.atualizar_portas).grid(
            row=0, column=2, padx=4
        )

        ttk.Label(quadro, text="Baudrate:").grid(row=0, column=3, padx=(12, 4))
        self.cb_baud = ttk.Combobox(
            quadro, width=10, values=BAUDRATES, state="readonly"
        )
        self.cb_baud.set(str(BAUD_PADRAO))
        self.cb_baud.grid(row=0, column=4, padx=4)

        self.btn_conectar = ttk.Button(
            quadro, text="Conectar", command=self.alternar_conexao
        )
        self.btn_conectar.grid(row=0, column=5, padx=8)
        self.lbl_status = ttk.Label(
            quadro, text="Desconectado", foreground="red"
        )
        self.lbl_status.grid(row=0, column=6, padx=4)

        chk_camera = ttk.Checkbutton(
            quadro,
            text="Espelhar meu rosto (câmera)",
            variable=self.espelhar_rostos,
            command=self._alternar_camera,
        )
        chk_camera.grid(row=0, column=7, padx=(20, 4))

    def _montar_presets(self):
        self.lf_presets = ttk.LabelFrame(self, text="Expressões (presets)")
        self.lf_presets.pack(fill="x", padx=8, pady=4)

        barra = ttk.Frame(self.lf_presets)
        barra.pack(fill="x", padx=4, pady=(4, 0))
        ttk.Label(barra, text="Duração da transição (ms):").pack(side="left")
        self.var_dur = tk.StringVar(value="400")
        ttk.Spinbox(
            barra, from_=0, to=5000, increment=50, width=6,
            textvariable=self.var_dur,
        ).pack(side="left", padx=4)
        ttk.Button(barra, text="Piscar", command=self.piscar).pack(
            side="left", padx=12
        )
        ttk.Button(
            barra,
            text="Salvar posição atual como preset...",
            command=self.salvar_preset,
        ).pack(side="left", padx=4)

        # Filtro passa-baixas ajustável: alpha baixo = mais suavização e atraso.
        linha_filtro = ttk.Frame(self.lf_presets)
        linha_filtro.pack(fill="x", padx=6, pady=(4, 2))
        ttk.Label(linha_filtro, text="Filtro facial (EMA):").pack(side="left")
        self.escala_filtro = tk.Scale(
            linha_filtro,
            from_=0.05,
            to=1.0,
            resolution=0.05,
            orient="horizontal",
            length=180,
            showvalue=False,
            variable=self.var_filtro_alpha,
            command=self._atualizar_rotulo_filtro,
        )
        self.escala_filtro.pack(side="left", padx=(6, 2))
        self.lbl_filtro = ttk.Label(linha_filtro, text="α = 0.30")
        self.lbl_filtro.pack(side="left", padx=4)
        ttk.Label(
            linha_filtro,
            text="Menor = mais suave; maior = mais rápido",
        ).pack(side="left", padx=(8, 0))

        self.frame_botoes = ttk.Frame(self.lf_presets)
        self.frame_botoes.pack(fill="x", padx=4, pady=4)

    def _atualizar_rotulo_filtro(self, _valor=None):
        """Atualiza o valor mostrado para o coeficiente do filtro EMA."""
        if hasattr(self, "lbl_filtro"):
            self.lbl_filtro.config(text=f"α = {self.var_filtro_alpha.get():.2f}")

    def _filtrar_alvos_faciais(self, alvos_pct):
        """Aplica filtro passa-baixas exponencial aos percentuais faciais.

        Na primeira amostra de cada canal, inicializa diretamente no valor
        detectado para evitar uma transição artificial partindo de zero.
        """
        alpha = limitar(float(self.var_filtro_alpha.get()), 0.05, 1.0)
        filtrados = {}
        for canal, alvo in alvos_pct.items():
            alvo = limitar(float(alvo), 0.0, 100.0)
            anterior = self._filtro_facial.get(canal)
            if anterior is None:
                filtrado = alvo
            else:
                filtrado = anterior + alpha * (alvo - anterior)
            filtrado = limitar(filtrado, 0.0, 100.0)
            self._filtro_facial[canal] = filtrado
            filtrados[canal] = round(filtrado)
        return filtrados

    def _desenhar_botoes_presets(self):
        for widget in self.frame_botoes.winfo_children():
            widget.destroy()

        todos = dict(PRESETS_PADRAO)
        todos.update(self.presets_custom)
        self.presets = todos

        for indice, nome in enumerate(todos):
            personalizado = nome in self.presets_custom
            texto = ("★ " if personalizado else "") + nome
            botao = ttk.Button(
                self.frame_botoes,
                text=texto,
                width=16,
                command=lambda n=nome: self.aplicar_preset(n),
            )
            botao.grid(
                row=indice // BOTOES_POR_LINHA,
                column=indice % BOTOES_POR_LINHA,
                padx=3,
                pady=3,
            )
            if personalizado:
                botao.bind("<Button-3>", lambda evento, n=nome: self.excluir_preset(n))
                botao.bind("<Button-2>", lambda evento, n=nome: self.excluir_preset(n))

    def _montar_servos(self):
        quadro = ttk.LabelFrame(self, text="Servos individuais")
        quadro.pack(fill="both", padx=8, pady=4)

        for canal, servo in enumerate(SERVOS):
            coluna = (canal // LINHAS_SERVOS) * 5
            linha = canal % LINHAS_SERVOS
            ttk.Label(
                quadro,
                text=f"{canal:02d}  {servo['nome']}",
                width=24,
                anchor="e",
            ).grid(
                row=linha, column=coluna, padx=(8, 4), pady=3, sticky="e"
            )

            slider = tk.Scale(
                quadro,
                from_=servo["min"],
                to=servo["max"],
                orient="horizontal",
                length=170,
                showvalue=False,
                resolution=1,
                command=lambda valor, ch=canal: self._slider_mudou(ch, valor),
            )
            slider.set((servo["min"] + servo["max"]) // 2)
            slider.grid(row=linha, column=coluna + 1, padx=4)

            variavel = tk.StringVar(value=str(slider.get()))
            entrada = ttk.Entry(
                quadro, textvariable=variavel, width=6, justify="center"
            )
            entrada.grid(row=linha, column=coluna + 2, padx=4)
            entrada.bind("<Return>", lambda evento, ch=canal: self._entrada_confirmada(ch))
            entrada.bind("<FocusOut>", lambda evento, ch=canal: self._entrada_confirmada(ch))

            ttk.Label(
                quadro,
                text=f"{servo['min']}–{servo['max']}",
                foreground="gray",
                width=9,
            ).grid(row=linha, column=coluna + 3, padx=(0, 14))

            self.sliders.append(slider)
            self.vars_entrada.append(variavel)
            self.entradas.append(entrada)

    def _montar_rodape(self):
        quadro = ttk.Frame(self)
        quadro.pack(fill="x", padx=8, pady=4)
        ttk.Button(quadro, text="Enviar todos", command=self.enviar_todos).pack(
            side="left", padx=4
        )
        ttk.Button(
            quadro, text="Soltar servos (OFF)", command=self.soltar_servos
        ).pack(side="left", padx=4)

        quadro_log = ttk.LabelFrame(self, text="Log / respostas do ESP32")
        quadro_log.pack(fill="both", padx=8, pady=(4, 8))
        self.txt_log = tk.Text(
            quadro_log, height=6, width=120, state="disabled", wrap="word"
        )
        self.txt_log.pack(side="left", fill="both", expand=True)
        barra = ttk.Scrollbar(quadro_log, command=self.txt_log.yview)
        barra.pack(side="right", fill="y")
        self.txt_log.config(yscrollcommand=barra.set)

    # ------------------------------- Câmera ------------------------------
    def _alternar_camera(self):
        if self.espelhar_rostos.get():
            self._iniciar_camera()
        else:
            self._desativar_camera()

    def _iniciar_camera(self):
        self._log("Inicializando câmera e carregando modelo MediaPipe...")
        try:
            if not ARQUIVO_MODELO.is_file():
                raise FileNotFoundError(
                    f"Modelo não encontrado: {ARQUIVO_MODELO}\n"
                    "Coloque face_landmarker.task na mesma pasta do programa."
                )

            opcoes_base = mp.tasks.BaseOptions(
                model_asset_path=str(ARQUIVO_MODELO)
            )
            opcoes = vision.FaceLandmarkerOptions(
                base_options=opcoes_base,
                output_face_blendshapes=True,
                num_faces=1,
            )
            detector = vision.FaceLandmarker.create_from_options(opcoes)
            captura = cv2.VideoCapture(0)
            if not captura.isOpened():
                captura.release()
                detector.close()
                raise RuntimeError("Não foi possível abrir a webcam.")

            self.detector = detector
            self.cap = captura
            self._filtro_facial.clear()
            self._ultimo_erro_camera = None
            self._log("Câmera ativada com sucesso!")
        except Exception as erro:
            self._desativar_camera(registrar=False)
            self.espelhar_rostos.set(False)
            messagebox.showerror(
                "Erro de câmera",
                f"Falha ao iniciar MediaPipe/câmera:\n{erro}",
                parent=self,
            )

    def _desativar_camera(self, registrar=True):
        self._filtro_facial.clear()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self.detector is not None:
            try:
                self.detector.close()
            except Exception:
                pass
            self.detector = None
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        if registrar and hasattr(self, "txt_log"):
            self._log("Câmera desativada.")

    @staticmethod
    def _extrair_alvos_faciais(shapes):
        """Converte blendshapes do MediaPipe em percentuais dos canais usados."""
        alvos = {}
        alvos[0] = round(shapes.get("jawOpen", 0.0) * 100)

        sobrancelha_esq = (
            50
            + round(shapes.get("browOuterUpLeft", 0.0) * 50)
            - round(shapes.get("browDownLeft", 0.0) * 50)
        )
        sobrancelha_dir = (
            50
            + round(shapes.get("browOuterUpRight", 0.0) * 50)
            - round(shapes.get("browDownRight", 0.0) * 50)
        )
        alvos[1] = limitar(sobrancelha_esq, 0, 100)
        alvos[5] = limitar(sobrancelha_dir, 0, 100)

        pálpebra_esq = round((1.0 - shapes.get("eyeBlinkLeft", 0.0)) * 100)
        pálpebra_dir = round((1.0 - shapes.get("eyeBlinkRight", 0.0)) * 100)
        alvos[4] = alvos[3] = limitar(pálpebra_dir, 0, 100)
        alvos[10] = alvos[11] = limitar(pálpebra_esq, 0, 100)

        olhar_horizontal = (
            50
            + round(shapes.get("eyeLookInLeft", 0.0) * 50)
            - round(shapes.get("eyeLookOutLeft", 0.0) * 50)
        )
        alvos[7] = alvos[12] = limitar(olhar_horizontal, 0, 100)

        olhar_vertical = (
            50
            + round(shapes.get("eyeLookUpLeft", 0.0) * 50)
            - round(shapes.get("eyeLookDownLeft", 0.0) * 50)
        )
        alvos[8] = alvos[13] = limitar(olhar_vertical, 0, 100)

        sorriso = round(
            max(
                shapes.get("mouthSmileLeft", 0.0),
                shapes.get("mouthSmileRight", 0.0),
            ) * 100
        )
        alvos[9] = alvos[14] = limitar(sorriso, 0, 100)
        alvos[15] = limitar(round(shapes.get("noseSneerLeft", 0.0) * 100), 0, 100)
        return alvos

    def _processar_camera(self):
        """Processa um quadro por ciclo, sem bloquear o loop do Tkinter."""
        try:
            if self.espelhar_rostos.get() and self.cap is not None and self.detector:
                sucesso, quadro = self.cap.read()
                if sucesso:
                    quadro = cv2.flip(quadro, 1)
                    rgb = cv2.cvtColor(quadro, cv2.COLOR_BGR2RGB)
                    imagem = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                    resultado = self.detector.detect(imagem)

                    if resultado.face_blendshapes:
                        blendshapes = resultado.face_blendshapes[0]
                        shapes = {
                            item.category_name: item.score for item in blendshapes
                        }
                        alvos_pct = self._extrair_alvos_faciais(shapes)
                        alvos_pct = self._filtrar_alvos_faciais(alvos_pct)
                        frame = [int(slider.get()) for slider in self.sliders]

                        for canal, percentual in alvos_pct.items():
                            if SERVOS[canal].get("livre"):
                                continue
                            tick = pct_para_tick(canal, percentual)
                            self._set_silencioso(canal, tick)
                            frame[canal] = tick

                        self._enviar_texto(",".join(map(str, frame)))

                    cv2.imshow("Rastreamento Facial InMoov", quadro)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.espelhar_rostos.set(False)
                        self._desativar_camera()
        except Exception as erro:
            # Evita interromper definitivamente o agendamento por um erro pontual.
            if str(erro) != self._ultimo_erro_camera:
                self._log(f"Erro no processamento da câmera: {erro}")
                self._ultimo_erro_camera = str(erro)
        finally:
            if not self._fechando:
                self.after(INTERVALO_CAMERA_MS, self._processar_camera)

    # --------------------------- Sliders / campos ------------------------
    def _slider_mudou(self, canal, valor):
        if self._atualizando:
            return
        tick = int(float(valor))
        self.vars_entrada[canal].set(str(tick))
        self.enviar_canal(canal, tick)

    def _entrada_confirmada(self, canal):
        servo = SERVOS[canal]
        try:
            valor = int(self.vars_entrada[canal].get().strip())
        except ValueError:
            self.vars_entrada[canal].set(str(self.sliders[canal].get()))
            return

        valor = limitar(valor, servo["min"], servo["max"])
        self._set_silencioso(canal, valor)
        self.enviar_canal(canal, valor)

    def _set_silencioso(self, canal, valor):
        self._atualizando = True
        try:
            valor = limitar(int(valor), SERVOS[canal]["min"], SERVOS[canal]["max"])
            self.sliders[canal].set(valor)
            self.vars_entrada[canal].set(str(valor))
        finally:
            self._atualizando = False

    # ------------------------------- Presets -----------------------------
    def aplicar_preset(self, nome, duracao=None):
        if self.espelhar_rostos.get():
            self._log("Desative o espelhamento da câmera para usar presets manuais.")
            return
        alvos = {}
        for canal, percentual in self.presets[nome].items():
            if not SERVOS[canal].get("livre"):
                alvos[canal] = pct_para_tick(canal, percentual)
        self.animar_para(alvos, duracao)
        self._log(f"Preset: {nome}")

    def animar_para(self, alvos, duracao=None, ao_terminar=None):
        if self._anim_id is not None:
            try:
                self.after_cancel(self._anim_id)
            except tk.TclError:
                pass
            self._anim_id = None

        if duracao is None:
            try:
                duracao = max(0, int(self.var_dur.get()))
            except ValueError:
                duracao = 400
        duracao = limitar(duracao, 0, 5000)

        inicio = {canal: int(self.sliders[canal].get()) for canal in alvos}
        passo_ms = 30
        passos = max(1, round(duracao / passo_ms))

        def passo(indice):
            proporcao = indice / passos
            # Interpolação suave (smoothstep).
            proporcao = proporcao * proporcao * (3 - 2 * proporcao)
            frame = [int(slider.get()) for slider in self.sliders]

            for canal, destino in alvos.items():
                valor = round(inicio[canal] + (destino - inicio[canal]) * proporcao)
                self._set_silencioso(canal, valor)
                frame[canal] = valor

            self._enviar_texto(",".join(map(str, frame)))
            if indice < passos:
                self._anim_id = self.after(passo_ms, lambda: passo(indice + 1))
            else:
                self._anim_id = None
                if ao_terminar:
                    ao_terminar()

        passo(1)

    def piscar(self):
        if self.espelhar_rostos.get():
            return
        canais = PALP_CIMA + PALP_BAIXO
        antes = {canal: int(self.sliders[canal].get()) for canal in canais}
        fechado = {canal: pct_para_tick(canal, 0) for canal in canais}
        self.animar_para(
            fechado,
            duracao=90,
            ao_terminar=lambda: self.animar_para(antes, duracao=140),
        )

    def salvar_preset(self):
        nome = simpledialog.askstring(
            "Salvar preset", "Nome do preset:", parent=self
        )
        if not nome or not nome.strip():
            return
        nome = nome.strip()
        if nome in PRESETS_PADRAO:
            messagebox.showwarning(
                "Nome reservado",
                "Escolha outro nome; esse nome pertence a um preset padrão.",
                parent=self,
            )
            return

        percentuais = {}
        for canal, servo in enumerate(SERVOS):
            if not servo.get("livre"):
                percentuais[canal] = tick_para_pct(
                    canal, int(self.sliders[canal].get())
                )
        self.presets_custom[nome] = percentuais
        try:
            self._gravar_custom()
        except OSError as erro:
            messagebox.showerror(
                "Erro ao salvar", f"Não foi possível salvar o preset:\n{erro}",
                parent=self,
            )
            return
        self._desenhar_botoes_presets()
        self._log(f"Preset '{nome}' salvo.")

    def excluir_preset(self, nome):
        if not messagebox.askyesno(
            "Excluir preset", f"Excluir o preset '{nome}'?", parent=self
        ):
            return
        self.presets_custom.pop(nome, None)
        try:
            self._gravar_custom()
        except OSError as erro:
            messagebox.showerror(
                "Erro ao salvar", f"Não foi possível atualizar os presets:\n{erro}",
                parent=self,
            )
            return
        self._desenhar_botoes_presets()
        self._log(f"Preset '{nome}' excluído.")

    def _carregar_custom(self):
        if not ARQUIVO_PRESETS.exists():
            return {}
        try:
            bruto = json.loads(ARQUIVO_PRESETS.read_text(encoding="utf-8"))
            personalizados = {}
            for nome, canais in bruto.items():
                if not isinstance(canais, dict):
                    continue
                limpos = {}
                for canal_txt, percentual in canais.items():
                    canal = int(canal_txt)
                    if 0 <= canal < NUM_SERVOS and not SERVOS[canal].get("livre"):
                        limpos[canal] = int(limitar(int(percentual), 0, 100))
                personalizados[str(nome)] = limpos
            return personalizados
        except (OSError, json.JSONDecodeError, TypeError, ValueError) as erro:
            print(f"Falha ao ler presets personalizados: {erro}")
            return {}

    def _gravar_custom(self):
        dados = {
            nome: {str(canal): percentual for canal, percentual in canais.items()}
            for nome, canais in self.presets_custom.items()
        }
        temporario = ARQUIVO_PRESETS.with_suffix(".tmp")
        temporario.write_text(
            json.dumps(dados, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        temporario.replace(ARQUIVO_PRESETS)

    # ---------------------------- Comunicação serial ---------------------
    def atualizar_portas(self):
        portas = [porta.device for porta in serial.tools.list_ports.comports()]
        self.cb_porta["values"] = portas
        if portas and self.cb_porta.get() not in portas:
            self.cb_porta.set(portas[0])
        elif not portas:
            self.cb_porta.set("")

    def alternar_conexao(self):
        if self.ser is not None and self.ser.is_open:
            self.desconectar()
        else:
            self.conectar()

    def conectar(self):
        porta = self.cb_porta.get()
        if not porta:
            messagebox.showwarning("Porta", "Selecione uma porta serial.", parent=self)
            return
        try:
            baudrate = int(self.cb_baud.get())
        except ValueError:
            messagebox.showwarning("Baudrate", "Baudrate inválido.", parent=self)
            return

        try:
            conexao = serial.Serial(porta, baudrate, timeout=0.1, write_timeout=0.5)
        except (serial.SerialException, OSError) as erro:
            messagebox.showerror("Erro ao abrir porta", str(erro), parent=self)
            self.ser = None
            return

        self.ser = conexao
        self._log(f"Conectado em {porta} @ {baudrate}. Aguardando reinício...")
        self.update_idletasks()
        time.sleep(2)  # tempo para placas que reiniciam ao abrir a serial
        try:
            self.ser.reset_input_buffer()
        except (serial.SerialException, OSError) as erro:
            self._log(f"Aviso ao limpar buffer serial: {erro}")

        self.leitor_ativo = True
        self._thread_rx = threading.Thread(target=self._thread_leitura, daemon=True)
        self._thread_rx.start()
        self.btn_conectar.config(text="Desconectar")
        self.lbl_status.config(text=f"Conectado ({porta})", foreground="green")

    def desconectar(self):
        self.leitor_ativo = False
        conexao = self.ser
        self.ser = None
        if conexao is not None:
            try:
                conexao.close()
            except (serial.SerialException, OSError):
                pass
        self.btn_conectar.config(text="Conectar")
        self.lbl_status.config(text="Desconectado", foreground="red")
        self._log("Desconectado.")

    def _thread_leitura(self):
        """Lê respostas da serial fora da thread da interface gráfica."""
        conexao = self.ser
        while self.leitor_ativo and conexao is not None and conexao.is_open:
            try:
                linha = conexao.readline()
            except (serial.SerialException, OSError) as erro:
                if self.leitor_ativo:
                    self.fila_rx.put(f"ERRO_SERIAL:{erro}")
                break
            if linha:
                self.fila_rx.put(linha.decode("utf-8", errors="replace").rstrip())

    def _processar_fila(self):
        try:
            while True:
                mensagem = self.fila_rx.get_nowait()
                if mensagem.startswith("ERRO_SERIAL:"):
                    self._log(mensagem.partition(":")[2], prefixo="Erro serial: ")
                elif not mensagem.startswith("OK "):
                    self._log(mensagem, prefixo="ESP32: ")
        except queue.Empty:
            pass
        finally:
            if not self._fechando:
                self.after(INTERVALO_FILA_MS, self._processar_fila)

    def _enviar_texto(self, texto):
        conexao = self.ser
        if conexao is None or not conexao.is_open:
            return False
        try:
            with self._lock_serial:
                conexao.write((texto + "\n").encode("utf-8"))
            return True
        except (serial.SerialException, OSError) as erro:
            self._log(f"Erro de envio: {erro}")
            return False

    # ------------------------------- Comandos ----------------------------
    def enviar_canal(self, canal, valor):
        frame = [int(slider.get()) for slider in self.sliders]
        frame[canal] = int(valor)
        if not self._enviar_texto(",".join(map(str, frame))):
            # Não registra cada movimento do slider para evitar encher o log.
            return

    def enviar_todos(self):
        frame = [int(slider.get()) for slider in self.sliders]
        texto = ",".join(map(str, frame))
        if self._enviar_texto(texto):
            self._log("TX: " + texto)
        else:
            self._log("Não conectado.")

    def soltar_servos(self):
        if self._enviar_texto("OFF"):
            self._log("TX: OFF")
        else:
            self._log("Não conectado.")

    # ------------------------------- Utilidades --------------------------
    def _log(self, mensagem, prefixo=""):
        if not hasattr(self, "txt_log") or not self.txt_log.winfo_exists():
            return
        self.txt_log.config(state="normal")
        self.txt_log.insert("end", prefixo + str(mensagem) + "\n")
        self.txt_log.see("end")
        linhas = int(self.txt_log.index("end-1c").split(".")[0])
        if linhas > 500:
            self.txt_log.delete("1.0", f"{linhas - 400}.0")
        self.txt_log.config(state="disabled")

    def _fechar(self):
        if self._fechando:
            return
        self._fechando = True
        self.espelhar_rostos.set(False)
        self._desativar_camera(registrar=False)
        self.leitor_ativo = False
        conexao = self.ser
        self.ser = None
        if conexao is not None:
            try:
                conexao.close()
            except (serial.SerialException, OSError):
                pass
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
