# Build and serve the React (Vite) dashboard
#
# Build args:
#   --build-arg VITE_ROSBRIDGE_URL=ws://<pi-ip>:9090
#   --build-arg VITE_API_BASE=http://<pi-ip>:8000
#   --build-arg VITE_MICROROS_HEARTBEAT_TOPIC=/microros/heartbeat
#

FROM node:20-bookworm-slim AS build

WORKDIR /app

# Install deps first (better layer caching)
COPY src/dashboard/package.json src/dashboard/package-lock.json ./
RUN npm ci

# Copy sources
COPY src/dashboard/ ./

# Build-time env for Vite (baked into the static bundle)
ARG VITE_ROSBRIDGE_URL=ws://localhost:9090
ARG VITE_API_BASE=http://localhost:8000
ARG VITE_MICROROS_HEARTBEAT_TOPIC=/microros/heartbeat
ENV VITE_ROSBRIDGE_URL=${VITE_ROSBRIDGE_URL}
ENV VITE_API_BASE=${VITE_API_BASE}
ENV VITE_MICROROS_HEARTBEAT_TOPIC=${VITE_MICROROS_HEARTBEAT_TOPIC}

RUN npm run build

FROM nginx:1.27-alpine
COPY --from=build /app/dist /usr/share/nginx/html
COPY docker/nginx-dashboard.conf /etc/nginx/conf.d/default.conf
EXPOSE 80


