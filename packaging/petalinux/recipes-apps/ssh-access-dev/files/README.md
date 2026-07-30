# Development SSH key input

This directory intentionally contains no `authorized_keys` file. Personal or
device-specific SSH identities must not be committed to the project.

For a local development image, pass an external public-key file explicitly:

```sh
CAM_SSH_AUTHORIZED_KEYS_FILE=/absolute/path/to/authorized_keys \
  scripts/petalinux/stage_ssh_access_dev.sh
```

The staging script validates the file with `ssh-keygen`, copies it only into
the local PetaLinux `meta-user` workspace, and prints fingerprints rather than
key material. The generated `debug` account remains password-locked.
