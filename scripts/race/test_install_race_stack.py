"""install-race-stack.sh's functions, sourced (the script stops before installing anything when
sourced): t_renderer download/fallback, the venv fallback, race-env.sh's SWB."""
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SCRIPT = os.environ.get("INSTALL_SH",
                        os.path.join(REPO, "setup", "scripts", "install-race-stack.sh"))

FAKE_TERA = '#!/bin/bash\necho "t_renderer 0.2.1"\n'


def _sh(tmp_path, body, env_extra=None):
    env = {k: v for k, v in os.environ.items() if k != "SWB"}
    env.update({"AUTOSDV_RACE_VENV": str(tmp_path / "venv"),
                "ACADOS_ROOT": str(tmp_path / "acados")})
    env.update(env_extra or {})
    prog = f'source "{SCRIPT}"\ntera_required() {{ echo 0.2.1; }}\n{body}\n'
    return subprocess.run(["bash", "-c", prog], env=env, capture_output=True, text=True,
                          timeout=300)


# ---- t_renderer ----

def _curl_ok(log):
    # writes the fake binary at -o FILE, logs the URL
    return ('curl() { local o; while [ $# -gt 0 ]; do case $1 in -o) o=$2; shift 2;; '
            f'-*) shift;; *) echo "$1" >> "{log}"; shift;; esac; done; '
            f"printf '%s' '{FAKE_TERA}' > \"$o\"; }}")


def test_arm64_downloads_the_release_binary(tmp_path):
    log = tmp_path / "urls"
    r = _sh(tmp_path, _curl_ok(log) + '\ncargo() { echo CARGO >&2; return 1; }\n'
            'TERA_ARCH=arm64\ninstall_tera')
    assert r.returncode == 0, r.stderr
    assert "CARGO" not in r.stderr
    assert log.read_text().strip().endswith("/v0.2.1/t_renderer-v0.2.1-linux-arm64")
    assert os.access(tmp_path / "acados" / "bin" / "t_renderer", os.X_OK)


def test_a_failed_download_falls_back_to_cargo(tmp_path):
    body = ('curl() { return 22; }\n'
            'cargo() { local root; while [ $# -gt 0 ]; do case $1 in --root) root=$2; shift 2;; '
            '*) shift;; esac; done; mkdir -p "$root/bin"; '
            f"printf '%s' '{FAKE_TERA}' > \"$root/bin/t_renderer\"; echo CARGO >&2; }}\n"
            'TERA_ARCH=arm64\ninstall_tera')
    r = _sh(tmp_path, body)
    assert r.returncode == 0, r.stderr
    assert "download failed" in r.stderr and "CARGO" in r.stderr
    assert os.access(tmp_path / "acados" / "bin" / "t_renderer", os.X_OK)


def test_no_download_and_no_cargo_names_rustup(tmp_path):
    r = _sh(tmp_path, 'curl() { return 22; }\nPATH=/usr/bin:/bin\nhash -r\n'
            'TERA_ARCH=arm64\ninstall_tera')
    assert r.returncode != 0
    assert "rustup" in r.stderr


# ---- venv (the no-uv fallback) ----

def test_a_venv_without_a_working_python_is_made_again(tmp_path):
    vbin = tmp_path / "venv" / "bin"
    vbin.mkdir(parents=True)
    (vbin / "python").write_text("#!/bin/sh\nexit 1\n")       # half made: present, does not run
    (vbin / "python").chmod(0o755)
    r = _sh(tmp_path, "have_uv() { return 1; }\nmake_venv")
    assert r.returncode == 0, r.stderr
    out = subprocess.run([str(vbin / "python"), "-c", "import sys; print(sys.base_prefix)"],
                         capture_output=True, text=True)
    assert out.returncode == 0 and out.stdout.strip() == "/usr"


def test_a_failing_venv_module_is_cleaned_up_with_the_apt_hint(tmp_path):
    fake = tmp_path / "fakebin"
    fake.mkdir()
    # a python whose venv makes bin/python and then fails, as Jammy's does without python3.10-venv
    py = fake / "python3.10"
    py.write_text('#!/bin/bash\nmkdir -p "$3/bin"; ln -sf /usr/bin/python3.10 "$3/bin/python"; '
                  'echo "ensurepip is not available" >&2; exit 1\n')
    py.chmod(0o755)
    r = _sh(tmp_path, f'have_uv() {{ return 1; }}\nresolve_py() {{ echo "{py}"; }}\nmake_venv')
    assert r.returncode != 0
    assert "python3.10-venv" in r.stderr
    assert not (tmp_path / "venv").exists()


def test_venv_never_uses_a_home_local_python(tmp_path):
    r = _sh(tmp_path, "resolve_py")
    assert r.returncode == 0, r.stderr
    assert r.stdout.strip() == "/usr/bin/python3.10"


# ---- race-env.sh ----

def test_race_env_keeps_the_install_time_swb_as_a_default(tmp_path):
    (tmp_path / "venv").mkdir()
    r = _sh(tmp_path, "write_env", env_extra={"SWB": "/install/swb"})
    assert r.returncode == 0, r.stderr
    env_file = tmp_path / "venv" / "race-env.sh"

    def swb_after_source(pre):
        return subprocess.run(["bash", "-c", f'{pre} source "{env_file}"; echo "$SWB"'],
                              capture_output=True, text=True).stdout.strip()

    assert swb_after_source("unset SWB;") == "/install/swb"
    assert swb_after_source("export SWB=/operator/swb;") == "/operator/swb"
