# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `8a76f7b`
- Tag: `deploy-8a76f7b`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=8a76f7b bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
