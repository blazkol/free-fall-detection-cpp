# free-fall-detection-cpp

## Prerequisites
![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)

## Getting Started

### 🔨 Building the Project

```bash
docker-compose build app
```

If you encounter build issues on Windows, try:

```bash
docker-compose build app --build-arg IS_WINDOWS=true
```

### 🚀 Running the Project

```bash
docker-compose run --rm --name free-fall-app app
```

## ⚙️ Configuration

You can change the project configuration by editing the `resources/config.ini` file.
