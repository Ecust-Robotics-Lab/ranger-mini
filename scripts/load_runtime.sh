# Source from project scripts after determining ROOT. Resolver output uses
# shlex.quote and a fixed whitelist of variable names; fail before any launch.
RANGER_RUNTIME_VALUES="$(python3 "$ROOT/scripts/runtime_config.py" --shell)" || return 2
eval "$RANGER_RUNTIME_VALUES"
unset RANGER_RUNTIME_VALUES
