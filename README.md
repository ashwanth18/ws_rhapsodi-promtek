# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `b5afac0`
- Tag: `deploy-b5afac0`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=b5afac0 bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
