#!/usr/bin/env bash
set -euo pipefail

# Ensure required tools are present (install at runtime to keep image clean)
need_tmux=0
need_nvim=0
command -v tmux >/dev/null 2>&1 || need_tmux=1
command -v nvim >/dev/null 2>&1 || need_nvim=1

if [[ $need_tmux -eq 1 || $need_nvim -eq 1 ]]; then
  echo "[dev-entrypoint] Installing packages: tmux=${need_tmux} nvim=${need_nvim}"
  apt-get update -y >/dev/null
  pkgs=()
  [[ $need_tmux -eq 1 ]] && pkgs+=(tmux)
  [[ $need_nvim -eq 1 ]] && pkgs+=(neovim)
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${pkgs[@]}" >/dev/null
  rm -rf /var/lib/apt/lists/*
fi

# Wire configs: prefer host's, fallback to repo defaults
mkdir -p /root/.config

# Neovim
if [[ -d /hosthome/.config/nvim ]] && [[ -n "$(ls -A /hosthome/.config/nvim 2>/dev/null || true)" ]]; then
  rm -rf /root/.config/nvim 2>/dev/null || true
  ln -s /hosthome/.config/nvim /root/.config/nvim
  echo "[dev-entrypoint] Using host nvim config"
elif [[ -d /work/defaults/nvim ]]; then
  rm -rf /root/.config/nvim 2>/dev/null || true
  cp -r /work/defaults/nvim /root/.config/nvim
  echo "[dev-entrypoint] Using default nvim config"
fi

# tmux
if [[ -f /hosthome/.tmux.conf ]]; then
  ln -sf /hosthome/.tmux.conf /root/.tmux.conf
  echo "[dev-entrypoint] Using host tmux.conf"
elif [[ -f /work/defaults/tmux/tmux.conf ]]; then
  cp -f /work/defaults/tmux/tmux.conf /root/.tmux.conf
  echo "[dev-entrypoint] Using default tmux.conf"
fi

# Drop into interactive shell
exec bash -li

