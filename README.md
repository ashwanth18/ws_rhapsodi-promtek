# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `6449a5f`
- Tag: `deploy-6449a5f`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=6449a5f bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
