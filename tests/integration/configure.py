from pathlib import Path
p=Path('/opt/cFS/sample_defs/targets.cmake')
p.write_text(p.read_text()+'\nlist(APPEND TGT1_APPLIST probe)\n')
p=Path('/opt/cFS/sample_defs/cpu1_cfe_es_startup.scr')
p.write_text('CFE_APP, /cf/probe.so, PROBE_Main, PROBE, 54, 32768, 0x0, 0;\n'+p.read_text())
