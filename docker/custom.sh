#!/bin/bash
curl -fsSL https://d2lang.com/install.sh | sh -s -- --dry-run
curl -fsSL https://d2lang.com/install.sh | sh -s --
export PATH=$HOME/.local/bin:$PATH