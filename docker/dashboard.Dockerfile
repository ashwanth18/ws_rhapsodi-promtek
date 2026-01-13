# Build and serve the React (Vite) dashboard
#
# Build arg:
#   --build-arg VITE_ROSBRIDGE_URL=ws://<pi-ip>:9090
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
ENV VITE_ROSBRIDGE_URL=${VITE_ROSBRIDGE_URL}

RUN npm run build

FROM nginx:1.27-alpine
COPY --from=build /app/dist /usr/share/nginx/html
EXPOSE 80


