# ZK4500 for login, lock screen and sudo (fprintd)

The PKGBUILD replaces the distribution's `libfprint` with libfprint 1.94.100 plus
the ZK4500 driver. All other drivers stay included. fprintd and `pam_fprintd`
come unchanged from the distribution repositories.

## 1. Build and install

```bash
cd packaging
makepkg -si                 # builds, pulls build deps (gobject-introspection),
                            # replaces libfprint (confirm the conflict with "y")
sudo pacman -S fprintd      # daemon + pam_fprintd
```

Stop any running test program (`zk4500_capture`, `img-capture`) first. Only one
process can use the device at a time.

## 2. Enroll a finger

```bash
fprintd-enroll -f right-index-finger     # touch repeatedly until "enroll-completed"
fprintd-verify                           # expect "verify-match"
```

Place the finger flat with normal pressure and lift it after each prompt.
If you get many "verify-no-match", delete and re-enroll (`fprintd-delete $USER`)
and vary the position slightly between touches.

## 3. PAM configuration

**Keep a second terminal with `sudo -s` open** while editing PAM files so you can
revert a mistake. All lines below are `sufficient`: if the fingerprint or token
fails or times out, PAM falls back to the password.

### Login and lock screen: token, then fingerprint, then password

Add this line to the files below. If you already use `pam_u2f.so`, put it
directly below that line, otherwise directly above `pam_unix.so`:

```
auth       sufficient   pam_fprintd.so
```

| File                           | Effect                     |
|--------------------------------|----------------------------|
| `/etc/pam.d/lightdm`           | LightDM login              |
| `/etc/pam.d/xfce4-screensaver` | unlocking the screen       |
| `/etc/pam.d/polkit-1` (optional, copy from `/usr/lib/pam.d/polkit-1`) | graphical admin prompts |

Example `/etc/pam.d/lightdm`:

```
#%PAM-1.0
auth       sufficient   pam_u2f.so authfile=/home/<user>/.config/Yubico/u2f_keys cue
auth       sufficient   pam_fprintd.so
auth       required     pam_unix.so try_first_pass nullok
...
```

LightDM: select the user in the greeter, then either touch the sensor directly or
press Enter on the empty password field and touch the sensor (depends on the greeter).

### sudo: hardware token locally, password over SSH (no fingerprint)

`pam_succeed_if.so rhost != ""` does **not** work for this. PAM does not
understand quotes, so the value is compared with the literal text `""`. The
condition is always true and the token is always skipped. On top of that, sudo
does not set `rhost` on Linux at all (`pam_rhost` is off by default), not even
over SSH.

Instead, `pam/pam-is-ssh` checks whether an `sshd` / `sshd-session` process is
among the ancestors:

```bash
sudo install -m 755 -o root -g root pam/pam-is-ssh /usr/local/bin/pam-is-ssh
```

`/etc/pam.d/sudo`:

```
#%PAM-1.0
auth       [success=1 default=ignore]  pam_exec.so quiet /usr/local/bin/pam-is-ssh
auth       sufficient   pam_u2f.so authfile=/home/<user>/.config/Yubico/u2f_keys cue
auth       required     pam_unix.so try_first_pass nullok
account    include      system-auth
session    include      system-auth
```

Test: `sudo -k; sudo true` locally (token prompt) and over SSH (password prompt).

Limitation: a tmux/screen session started locally and attached later over SSH
counts as local, so you get the token prompt there.

## Notes

- Security: an optical sensor with the NBIS matcher is convenient but weaker than
  a password or a hardware token. Optical sensors can be fooled with fake prints.
- When the distribution ships a newer libfprint, this package is not replaced
  automatically. Bump `pkgver` in the PKGBUILD and rebuild if needed.
- Back to the stock package: `sudo pacman -S libfprint`.
- Debugging: `sudo systemctl stop fprintd; sudo G_MESSAGES_DEBUG=all /usr/lib/fprintd -t`
