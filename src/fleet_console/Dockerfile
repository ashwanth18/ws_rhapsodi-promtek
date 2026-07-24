# Multi-stage: build Fleet Console UI, then run FastAPI serving the static build.
FROM node:22-bookworm-slim AS web-build
WORKDIR /web
COPY web/package.json web/package-lock.json ./
RUN npm ci
COPY web/ ./
# vite.config.ts outDir is ../static → /static in this stage
RUN npm run build

FROM python:3.12-slim

WORKDIR /app
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV FLEET_DATA_DIR=/data
ENV FLEET_LOG_DIR=/data/logs
ENV REPO_ROOT=/repo
ENV ANSIBLE_DIR=/repo/ansible

RUN apt-get update && apt-get install -y --no-install-recommends \
  openssh-client \
  git \
  ansible \
  curl \
  ca-certificates \
  && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt

COPY app /app/app
COPY --from=web-build /static /app/static

EXPOSE 8090
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8090"]
