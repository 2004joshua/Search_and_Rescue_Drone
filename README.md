# Search_and_Rescue_Drone

multi_device_sim/
├── models/
│   └── Mobula6/                      # drone model
│       ├── frame.stl
│       ├── canopy.stl
│       └── Mobula6.proto
│
├── worlds/
│   └── multi_device_world.wbt        # defines Beacon, Drone & GroundStation robots
│
├── controllers/                      # one folder per “device”
│   ├── beacon/
│   │   ├── beacon.py                 # emits rescue beacon via Emitter
│   │   ├── requirements.txt          # e.g. webots
│   │   └── Dockerfile                # (optional) containerize it
│   │
│   ├── drone/
│   │   ├── drone.py                  # RSSI search logic, RX/TX handlers
│   │   ├── requirements.txt
│   │   └── Dockerfile
│   │
│   └── ground/
│       ├── ground.py                 # listens to drone, sends commands
│       ├── requirements.txt
│       └── Dockerfile
│
├── scripts/                          # utility scripts
│   └── build_controllers.sh          # e.g. build your Docker images
│
├── docker-compose.yml                # spin up all three controllers + Webots
│
├── README.md                         # overview, how to launch
└── .gitignore
