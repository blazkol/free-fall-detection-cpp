# Stage 1: Build Environment - use the official GCC image as the base
FROM gcc:latest AS builder

# Accept build argument to detect if host is Windows
ARG IS_WINDOWS=false

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

# Convert entrypoint script line endings if host is Windows
RUN if [ "$IS_WINDOWS" = "true" ]; then \
        apt-get update && apt-get install -y dos2unix && \
        dos2unix entrypoint.sh; \
    fi

# Stage 2: Runtime environment - use the official Ubuntu image as the base image
FROM ubuntu:latest

WORKDIR /app

# Copy the compiled binaries, entrypoint script and resources
COPY --from=builder /app/build ./build
COPY --from=builder /app/entrypoint.sh .
COPY ./resources ./resources

# Make the script executable 
RUN chmod +x entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["./entrypoint.sh"]