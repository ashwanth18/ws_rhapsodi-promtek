# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `c1ecc5c`
- Tag: `deploy-c1ecc5c`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=c1ecc5c bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
