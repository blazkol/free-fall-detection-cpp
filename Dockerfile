# Stage 1: Build Environment - use the official GCC image as the base
FROM gcc:latest AS builder

# Install necessary dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libboost-all-dev \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory and copy the source code
WORKDIR /app
COPY . /app

# Create the build directory and build the project
RUN mkdir build && cd build && \
    cmake .. && \
    make

# Stage 2: Runtime environment - use the official Ubuntu image as the base image
FROM ubuntu:latest

WORKDIR /app

# Copy the compiled binaries, data folder and entrypoint script
COPY --from=builder /app/build ./build
COPY ./data ./data
COPY ./entrypoint.sh .

# Make the script executable 
RUN chmod +x entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/app/entrypoint.sh"]