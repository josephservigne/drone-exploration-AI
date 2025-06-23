# AI Drone Exploration - FUEL Algorithm (Simplified Version)

**The project focuses on the autonomous exploration in 2D of a virtual environment simulated in PyBullet by a drone in Python.**

## Context

This project was developed as part of a semester-long academic project at CentraleSupelec. Our client was the start-up **Drakkair**, composed of three students/researchers:  
- Nicolas Rosal (CEO)  
- Clément Mauget (CTO)  
- Kenzy Barakat (intern and our main contact, currently in the 3rd year at CentraleSupélec)

The goal was to support Drakkair in developing two algorithms: **FUEL** and **DPPM**. Although the project involved both, **this repository only contains the FUEL algorithm**, as it is the one I contributed to directly. The related paper can be found as `FUEL.pdf` in the repository.

While a public GitHub repository existed for the FUEL implementation, we decided to start from scratch due to the poor structure of that codebase (+ it was mostly written in C). Additionally, we chose to **restrict the environment to two dimensions** to simplify data processing.

The result is a **clean, understandable, and maintainable version of the FUEL algorithm**, designed to serve as a first deliverable for our client.

⚠️ This project is still under development. In particular, the current implementation handles sensor noise poorly, which affects the robustness and performance of the exploration algorithm.


---

## 📁 Project Structure

```
exploration/
│
├── drone.py         # Defines the drone behavior and actions
├── first_test.py    # Main script containing the exploration loop
├── FIS.py           # Simplified data structure for frontier-based exploration
└── sandbox.py       # 2D virtual environment where the drone operates

requirements.txt     # List of Python packages to install
Rapport.pdf          # Final report explaining both FUEL and DPPM algorithms
FUEL.pdf             # Original paper describing the FUEL algorithm
```

---

## Getting Started

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Run the Simulation

```bash
python exploration/first_test.py
```

---

## Credits

Developed as part of a team project by 4 students (2 per algorithm) at CentraleSupelec.  
Client: Drakkair
