import datetime
import os.path as osp
import time
from pathlib import Path
from git import Repo
from types import ModuleType

from .command_utils import run_command


def git_available():
    return run_command("git version") is not None


def build_repo(repo):
    try:
        if isinstance(repo, ModuleType):
            repo = repo.__file__
        if isinstance(repo, (str, Path)):
            repo = Repo(str(repo), search_parent_directories=True)
    except:
        return None
    return repo if isinstance(repo, Repo) else None


def get_git_unstashed(repo):
    # Note that paths returned by git ls-files are relative to the script.
    # return run_command(f"cd {repo} && git ls-files {git_dir} -m")
    repo = build_repo(repo)
    return [_.a_path for _ in repo.index.diff(None)] if repo is not None else []


def get_git_untracked(repo):
    repo = build_repo(repo)
    return repo.untracked_files if repo is not None else []
    # Note that paths returned by git ls-files are relative to the script.
    # return run_command(f"cd {repo} && git ls-files {git_dir} --exclude-standard --others")


def get_git_modified(repo):
    # Note that paths returned by git ls-files are relative to the script.
    # return run_command(f"cd {repo} && git ls-files {git_dir} -m")
    repo = build_repo(repo)
    return get_git_untracked(repo), get_git_unstashed(repo)


def get_git_hash(repo, digits=None):
    repo = build_repo(repo)
    if repo is None:
        return None
    commit = str(repo.head.commit)
    return commit[:digits] if digits is not None else commit


def get_git_branch(repo):
    repo = build_repo(repo)
    if repo is None:
        return None
    return repo.active_branch.name


def get_git_repo_info(repo, digits=7):
    repo = build_repo(repo)
    if repo is None:
        return None
    branch = repo.active_branch.name
    head = repo.head
    commit = str(head.commit)
    commit_time = time.gmtime(head.commit.committed_date - 7 * 3600)  # CA time
    commit_time = time.strftime("%Y-%m-%d-%H:%M:%S", commit_time)
    commit = commit[:digits] if digits is not None else commit
    return branch, commit_time, commit


"""
def find_git_parent_folder(repo, name_only=False):
    repo = Path(repo)
    while str(repo) != "/":
        git_object = repo / ".git"
        if git_object.exists():
            if not name_only:
                git_object = Repo(git_object)
            return git_object
        repo = repo.parent
    return None
"""


def auto_push(repo, remote="origin", local_branch="HEAD", remote_branch=None):
    repo = build_repo(repo)
    if repo is None:
        return

    if remote_branch is None:
        remote_branch = get_git_branch(repo.working_dir)

    from pyrl.utils.meta.timer import get_time_stamp

    repo.git.add(".")
    repo.index.commit(f"Auto-commit at {get_time_stamp()}")
    repo.git.push(remote, f"{local_branch}:{remote_branch}")
