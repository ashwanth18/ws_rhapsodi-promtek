# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `0b305ef`
- Tag: `deploy-0b305ef`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=0b305ef bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
