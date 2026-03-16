#!/bin/bash
git co master
git merge upstream/master
push origin master
echo "now rebase ruedbi1 on master"
git co ruedbi1
git rebase master
git push origin master
echo "done"