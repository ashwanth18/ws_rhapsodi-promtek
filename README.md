# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `b40ea9b`
- Tag: `deploy-b40ea9b`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=b40ea9b bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
