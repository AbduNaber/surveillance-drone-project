# -*- mode: python ; coding: utf-8 -*-

block_cipher = None

a = Analysis(
    ['src/YoloWorker.py'],
    pathex=['src'],
    binaries=[],
    datas=[],  # DO NOT bundle .pt files here
    hiddenimports=[
        'torch',
        'torchvision',
        'cv2',
        'numpy'
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'matplotlib',
        'scipy',
        'pandas',
        'tkinter',
        'pytest',
    ],
    noarchive=True,     # 🔥 CRITICAL FIX
    optimize=0,
)

pyz = PYZ(a.pure, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='YoloWorker',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,          # 🔥 CRITICAL FIX
    console=True,
)
