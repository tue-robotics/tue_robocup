#!/usr/bin/env bash
#
# Check out the feature/llm branches that GPSR with the LLM task planner needs.
#
# All repositories are checked first. When one of them has work that a checkout could lose or hide (uncommitted
# changes, commits that are not on GitHub, a detached HEAD) or has no feature/llm branch, nothing is checked out: the
# problems are listed, fix them and run the script again.
#
#   checkout_llm_branches.sh            check, then check out feature/llm everywhere
#   checkout_llm_branches.sh --check    only check and report
#
# The repositories are looked for in $LLM_REPOS_DIR, default <tue-env workspace>/../repos/github.com/tue-robotics.
# Afterwards run tue-make.

set -u

BRANCH="feature/llm"
REPOS=(
    action_server         # the LLM task planner (action_server.llm)
    conversation_engine   # parser: llm
    tue_robocup           # GPSR, robocup_knowledge (object list, detector labels), Find
    hero_bringup          # planner instructions, parser setting, rgbd plugin configuration
    ed_sensor_integration # detector labels into the object list, vote per entity
    ed_python             # Entity.existence_probability
    yolo_onnx_ros         # configurable confidence threshold
    robot_launch_files    # writes the detector label map before ED is configured
)

CHECK_ONLY=false
case "${1:-}" in
    --check) CHECK_ONLY=true ;;
    "") ;;
    -h|--help) sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown argument '$1'. Use --check or no argument." >&2; exit 2 ;;
esac

if [[ -n "${LLM_REPOS_DIR:-}" ]]; then
    REPOS_DIR="$LLM_REPOS_DIR"
elif [[ -n "${TUE_ENV_WS_DIR:-}" ]]; then
    REPOS_DIR="$(dirname "$TUE_ENV_WS_DIR")/repos/github.com/tue-robotics"
else
    echo "Set LLM_REPOS_DIR, or source tue-env (TUE_ENV_WS_DIR)." >&2
    exit 2
fi

red() { echo -e "\e[31;1m$*\e[0m"; }
green() { echo -e "\e[32;1m$*\e[0m"; }
yellow() { echo -e "\e[33;1m$*\e[0m"; }

PROBLEMS=()
WARNINGS=()

# Fetch first, so that 'not on GitHub' and 'behind' are judged against the current remote
for repo in "${REPOS[@]}"; do
    dir="$REPOS_DIR/$repo"
    if [[ ! -d "$dir/.git" && ! -f "$dir/.git" ]]; then
        PROBLEMS+=("$repo: no git repository at $dir")
        continue
    fi
    git -C "$dir" fetch --quiet origin 2>/dev/null || WARNINGS+=("$repo: could not fetch origin; checked against the last fetch")
done

for repo in "${REPOS[@]}"; do
    dir="$REPOS_DIR/$repo"
    [[ -d "$dir/.git" || -f "$dir/.git" ]] || continue
    git() { command git -C "$dir" "$@"; }

    current=$(git symbolic-ref --quiet --short HEAD)
    if [[ -z "$current" ]]; then
        PROBLEMS+=("$repo: detached HEAD at $(git rev-parse --short HEAD); check out a branch")
        continue
    fi

    # Uncommitted changes to tracked files. Submodules that only point at another commit (docs) are not edits.
    changes=$(git status --porcelain --ignore-submodules=all --untracked-files=no)
    if [[ -n "$changes" ]]; then
        PROBLEMS+=("$repo ($current): uncommitted changes, commit or stash them:"$'\n'"$(sed 's/^/        /' <<< "$changes")")
    fi
    untracked=$(git status --porcelain --ignore-submodules=all | grep -c '^??')
    if [[ "$untracked" -gt 0 ]]; then
        WARNINGS+=("$repo ($current): $untracked untracked file(s); they stay where they are")
    fi

    # Commits that are only on this machine. On feature/llm itself that is the work in progress, so only a warning.
    upstream=$(git rev-parse --abbrev-ref --symbolic-full-name '@{upstream}' 2>/dev/null)
    if [[ -n "$upstream" ]]; then
        unpushed=$(git rev-list --count "$upstream..HEAD")
    else
        # No upstream: the commits that are on no remote branch
        unpushed=$(git rev-list --count HEAD --not --remotes=origin)
    fi
    if [[ "$unpushed" -gt 0 ]]; then
        if [[ "$current" == "$BRANCH" ]]; then
            WARNINGS+=("$repo ($current): $unpushed commit(s) not pushed")
        else
            PROBLEMS+=("$repo ($current): $unpushed commit(s) not on GitHub (${upstream:-no upstream}); push them or move them to a branch you keep")
        fi
    fi

    # The branch must exist
    if ! git show-ref --verify --quiet "refs/heads/$BRANCH" && ! git show-ref --verify --quiet "refs/remotes/origin/$BRANCH"; then
        PROBLEMS+=("$repo: no branch $BRANCH, locally or on GitHub")
    fi

    unset -f git
done

echo
for w in "${WARNINGS[@]}"; do yellow "warning: $w"; done
if [[ ${#PROBLEMS[@]} -gt 0 ]]; then
    red "Nothing was checked out. Fix these and run the script again:"
    for p in "${PROBLEMS[@]}"; do echo "  - $p"; done
    exit 1
fi

if $CHECK_ONLY; then
    green "All ${#REPOS[@]} repositories can switch to $BRANCH (--check: nothing changed)."
    exit 0
fi

# All checks passed: check out and update
FAILED=()
for repo in "${REPOS[@]}"; do
    dir="$REPOS_DIR/$repo"
    git() { command git -C "$dir" "$@"; }
    if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
        git checkout --quiet "$BRANCH" || { FAILED+=("$repo: checkout"); unset -f git; continue; }
        if git rev-parse --verify --quiet "origin/$BRANCH" >/dev/null; then
            git branch --quiet --set-upstream-to="origin/$BRANCH" 2>/dev/null
            git merge --quiet --ff-only "origin/$BRANCH" 2>/dev/null \
                || WARNINGS+=("$repo: $BRANCH and origin/$BRANCH have diverged; not updated")
        fi
    else
        git checkout --quiet --track "origin/$BRANCH" || { FAILED+=("$repo: checkout"); unset -f git; continue; }
    fi
    git submodule --quiet update --init 2>/dev/null || WARNINGS+=("$repo: submodule update failed")
    printf "  %-22s %s @ %s\n" "$repo" "$BRANCH" "$(git log -1 --format='%h %s' | cut -c1-60)"
    unset -f git
done

for w in "${WARNINGS[@]}"; do yellow "warning: $w"; done
if [[ ${#FAILED[@]} -gt 0 ]]; then
    red "Some checkouts failed:"
    for f in "${FAILED[@]}"; do echo "  - $f"; done
    exit 1
fi
green "All repositories are on $BRANCH. Now run: tue-make"
