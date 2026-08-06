# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `3ff4db7`
- Tag: `deploy-3ff4db7`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=3ff4db7 bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
