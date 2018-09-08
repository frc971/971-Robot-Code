# How to submit code to gerrit for review

## Prerequisites
1. Generate an ssh key and add it to Gerrit.

First, run `ssh-keygen` in your terminal, and accept all of the default
parameters by hitting enter. Then go to Gerrit, click on your name in the top
right corner and click settings. Choose `SSH Public Keys` on the side bar and
click `Add Key`. Copy paste the contents of `~/.ssh/id_rsa.pub` into the box and
hit okay.

2. Add the gerrit change-id git hook to your local repository

If you used the `clone with commit-msg hook` command when you first
cloned the code, you can skip this step. If you did not, run the following
commands from the 971-Robot-Code directory, replacing `<username>` with your
gerrit username.

```console
scp -p -P 29418 <username>@robotics.mvla.net:hooks/commit-msg .git/hooks/
```

## Submitting code for review

1. Choose what changes you want to commit
  - You can see what files you've changed using `git status`
  - You can use `git diff` to see exactly what you changed. Note that this will
    not show you new files that you've created locally.

2. Once you know what you want to commit, use `git add <file>` to "stage" them
   for the commit. You should only add the changes that you want, so if your
   files contain things you don't want to commit, you'll have to cut out the
   stuff you don't want before using `git add`.

3. After you've used `git add` to add all the changes you want to commit, run
   `git commit` and write an informative commit message in the editor that pops
   up. Save and quit in the editor that opens to actually commit your changes.
   If you change your mind about committing, you can quit without saving to
   cancel the commit.
   - You can use `git commit -v` to see what changes `git` thinks you want to
     commit in the commit message editor.
   - If your commit message is short, you can use `git commit -m "<message>"` to
     specify it on the command line.

4. Send your changes to Gerrit by running
   `git push origin HEAD:refs/for/master`.
   - A link to the Gerrit web interface will be printed for you to copy paste
     into your browser.

5. Add reviewers to your change request so that your code can be reviewed and
   approved after feedback.

## Editing your submitted change

Gerrit allows you to edit your change and resubmit it.

1. Make sure that the commit you want to edit is the latest in your commit
   history by running `git show` and making sure that the commit is the one you
   want to edit.

2. Make the changes you want to make.

3. Add the changes using `git add <file1> <file2> ...`.

4. Add the changes you made to the previous commit by running
   `git commit --amend`. This will allow you to edit the previous commit
   message. MAKE SURE YOU DON'T EDIT THE GERRIT Change-Id. This is what Gerrit
   uses to know that you're updating an existing change instead of pushing a new
   one.

## Troubleshooting
* If your commit is missing a Change-Id, you'll get a reasonably understandable
  error that looks like the following:
```console
Counting objects: 5, done.
Delta compression using up to 4 threads.
Compressing objects: 100% (5/5), done.
Writing objects: 100% (5/5), 2.83 KiB | 0 bytes/s, done.
Total 5 (delta 1), reused 0 (delta 0)
remote: Resolving deltas: 100% (1/1)
remote: Processing changes: refs: 1, done
remote: ERROR: [bf965b4] missing Change-Id in commit message footer
remote:
remote: Hint: To automatically insert Change-Id, install the hook:
remote:   gitdir=$(git rev-parse --git-dir); scp -p -P 29418 adrian@robotics.mvla.net:hooks/commit-msg ${gitdir}/hooks/
remote: And then amend the commit:
remote:   git commit --amend
remote:
```
  If you see this, run the commands that they suggest in the error message and
  push it again
