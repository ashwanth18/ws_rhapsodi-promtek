# Jaka site-2 layout profile

Override poses / named targets / container scene for a different physical
table layout. Edit these YAML files for the site-2 robot, then redeploy with:

```bash
ansible-playbook ansible/deploy.yml --limit <device> \
  -e image_tag=<sha> -e profile=jaka-site2-layout
```

Or pick **jaka-site2-layout** in the Fleet Console deploy panel.

Starting copies are placeholders from the default Niryo/Jaka seeds — re-record
from RViz Save on the real cell before production use.
