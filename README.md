# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `bea8e0a`
- Tag: `deploy-bea8e0a`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=bea8e0a bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
