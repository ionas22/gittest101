# DOFBOT Cube Robot

Aplicație Python pentru Raspberry Pi 4 + Yahboom DOFBOT și cameră USB.

## Instalare

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

Adaptorul folosește API-ul uzual `Arm_Lib` + `ArmIK` din imaginea oficială
Yahboom. Dacă imaginea instalată expune alte denumiri, modificați doar
`dofbot_robot/arm.py`.

## Utilizare

```bash
python3 -m dofbot_robot --mode calibrate
python3 -m dofbot_robot --mode reproduce --config config/robot.json
python3 -m dofbot_robot --mode tower --order red,yellow,blue,green --target 2,2
```

Înainte de prima rulare:

1. fixați camera deasupra celor două caroiaje;
2. rulați calibrarea și salvați coordonatele colțurilor;
3. verificați pozițiile de preluare/depunere la viteze mici;
4. testați cu `--dry-run`.

Sistemul folosește culoarea feței superioare ca orientare. Fețele laterale
cu imagini nu sunt folosite la clasificare, dar cubul este transportat fără
rotire intenționată.
