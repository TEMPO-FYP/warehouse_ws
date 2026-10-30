# Warehouse Workspace (`warehouse_ws`)

A unified multi-package ROS 2 workspace and simulation environment built for **ROS 2 Jazzy** and **Gazebo Sim (Harmonic)**. Designed as a team monorepo: all ROS 2 packages live inside `src/` and are mounted directly into a single, high-performance Docker container.

Includes:
- **`warehouse_sim`**: 25+ modular warehouse models (shelves, pallet racks, forklifts, pallets, barrels), an SDF world, and automated `/clock` bridging for navigation and manipulation tasks.
- Extensible workspace structure ready for additional packages (`tempo_description`, `tempo_navigation`, `tempo_planner`, etc.).

---

## Prerequisites (Host Machine)

### 1. Install Docker & Docker Compose
If Docker is not already installed on your Ubuntu system:

```bash
# 1. Uninstall any conflicting legacy packages:
sudo apt remove $(dpkg --get-selections docker.io docker-compose docker-compose-v2 docker-doc docker-buildx podman-docker containerd runc 2>/dev/null | cut -f1)

# 2. Set up Docker's official apt repository:
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add repository to Apt sources (deb822 format):
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update

# 3. Install Docker Engine and plugins:
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 4. Verify installation:
sudo docker run hello-world

# 5. Configure permissions and enable system services:
sudo groupadd docker 2>/dev/null || true
sudo usermod -aG docker $USER
sudo systemctl enable docker.service
sudo systemctl enable containerd.service

# Note: Reboot or run 'newgrp docker' to apply group changes:
# sudo reboot
```

---

## Quick Start (Docker)

### 1. Clone the Repository
```bash
git clone https://github.com/TEMPO-FYP/warehouse_ws.git
cd warehouse_ws/docker
```

### 2. Build the Docker Image
```bash
bash build.sh
```

### 3. Grant X11 Display Permission (Once per login session)
On your host machine, allow the Docker container to display GUI windows:
```bash
xhost +
```

### 4. Start the Container
```bash
docker compose up -d warehouse_ws
```

### 5. Enter Container & Launch the Simulation
```bash
# Open interactive shell in the container
docker compose exec warehouse_ws bash

# Launch Gazebo Sim with the warehouse world and clock bridge
ros2 launch warehouse_sim warehouse.launch.py
# (or use the built-in alias)
sim
```

### 6. Stop the Container
When finished, stop the container from the `docker/` directory on your host machine:
```bash
docker compose stop warehouse_ws
```
*(To completely remove the container: `docker compose down`)*

---

## Adding New Packages

To add a new package (e.g. robot description, navigation):
1. Create or place your package inside `warehouse_ws/src/` on your host machine.
2. Because `src/` is live-mounted into the container, it is immediately accessible at `/root/warehouse_ws/src/`.
3. Inside the container, re-build with:
   ```bash
   build
   ```

---

## Useful Shortcuts & Aliases (Inside Container)

| Shortcut | Command | Description |
| :--- | :--- | :--- |
| `sim` | `ros2 launch warehouse_sim warehouse.launch.py` | Launches the warehouse world |
| `build` | `cd /root/warehouse_ws && colcon build --symlink-install && source install/setup.bash` | Rebuilds the workspace |
| `ws` | `cd /root/warehouse_ws` | Jump to workspace root |
| `src` | `cd /root/warehouse_ws/src` | Jump to workspace source directory |
| `cleanup` | Kills lingering ROS/Gazebo processes | Resets ROS/Gazebo processes |

---

## Repository Structure

```text
warehouse_ws/
├── README.md                 # Setup & team workflow guide
├── .gitignore
├── docker/                   # Project-wide Docker environment
│   ├── Dockerfile            # ROS 2 Jazzy base with Mesa and dependencies
│   ├── docker-compose.yml    # Host networking, X11 forwarding, GPU/dev access
│   ├── build.sh              # Build automation script
│   ├── workspace.sh          # Dependency installation & workspace setup
│   ├── entrypoint.sh         # Container entrypoint
│   └── bash_aliases.txt      # Shell productivity shortcuts
└── src/                      # ROS 2 packages live here (mounted live into container)
    └── warehouse_sim/        # Warehouse simulation environment package
        ├── package.xml       # ROS 2 package manifest
        ├── setup.py          # Installation logic for worlds and models
        ├── setup.cfg
        ├── LICENSE           # Apache 2.0 License
        ├── resource/
        ├── warehouse_sim/    # Python module directory
        ├── launch/
        │   └── warehouse.launch.py
        ├── worlds/
        │   └── warehouse.sdf
        └── models/           # 25+ modular models
```
