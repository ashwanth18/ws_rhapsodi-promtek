# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `89773b2`
- Tag: `deploy-89773b2`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=89773b2 bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
