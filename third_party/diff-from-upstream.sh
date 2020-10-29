#!/bin/bash

# This script will run `git diff` from the latest upstream to HEAD on the
# git-subtreed directory given as its first argument, passing all of the
# other arguments on.
#
# This will not work on non-squashed git-subtree directories.

# Copied from
# https://github.com/git/git/blob/master/contrib/subtree/git-subtree.sh.
find_latest_squash()
{
	dir="$1"
	sq=
	main=
	sub=
	git log --grep="^git-subtree-dir: $dir/*\$" \
		--pretty=format:'START %H%n%s%n%n%b%nEND%n' HEAD |
	while read a b junk; do
		case "$a" in
			START) sq="$b" ;;
			git-subtree-mainline:) main="$b" ;;
			git-subtree-split:) sub="$b" ;;
			END)
				if [ -n "$sub" ]; then
					if [ -n "$main" ]; then
						# a rejoin commit?
						# Pretend its sub was a squash.
						sq="$sub"
					fi
					echo "$sq" "$sub"
					break
				fi
				sq=
				main=
				sub=
				;;
		esac
	done
}

DIR="${1%/}"
shift
DIFF_ARGS="$@"

SPLIT="$(find_latest_squash "${DIR}")"
if [ -z "${SPLIT}" ]; then
	echo "${DIR} does not appear to be git-subtreed in."
	exit 1
fi

set ${SPLIT}
SQUASHED_UPSTREAM=$1

git diff ${DIFF_ARGS} ${SQUASHED_UPSTREAM}..HEAD:${DIR}
