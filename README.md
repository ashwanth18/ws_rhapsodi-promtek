# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: `c0bbc35`
- Tag: `deploy-c0bbc35`
- Do **not** develop here — edit files on `main` and re-run
  `scripts/publish_deploy_bundle.sh`.

Provision:
```bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=c0bbc35 bash scripts/provision_device.sh
```

(The provision script lives on `main`; devices only need this branch.)
