# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['simulator.py'],
    pathex=[],
    binaries=[('C:\\Users\\seong\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\DLLs\\_tkinter.pyd', '.'), ('C:\\Users\\seong\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\DLLs\\tcl86t.dll', '.'), ('C:\\Users\\seong\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\DLLs\\tk86t.dll', '.')],
    datas=[('C:\\Users\\seong\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\Lib\\tkinter', 'tkinter'), ('C:\\Users\\seong\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\tcl', 'tcl')],
    hiddenimports=['serial'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='MecanumSensorSimulator',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
