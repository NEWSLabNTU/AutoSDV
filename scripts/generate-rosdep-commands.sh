#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
rosdep keys --from-paths src --ignore-src | \
    sort | \
    while read key; do
	echo "rosdep resolve $key 2>/dev/null"
    done | \
	parallel --group | \
	awk '\
BEGIN {
  mode = ""
  n_apt = 0
  n_pip = 0
 }

/^#apt$/ { mode = "apt" }
/^#pip$/ { mode = "pip" }

/^[^#]/ {
  if (mode == "apt") {
    apt[n_apt] = $0
    n_apt += 1
  } else if (mode == "pip") {
    pip[n_pip] = $0
    n_pip += 1
  } else {
    print "Error: Expect #apt or #pip before a key" > "/dev/stderr"
    exit 1
  }
}

END {
  print "#!/usr/bin/env bash"
  printf "sudo apt install -y"
  for (i = 0; i < n_apt; i += 1) {
        printf " %s", apt[i]
  }
  printf "\n"

  printf "pip install -U"
  for (i = 0; i < n_pip; i += 1) {
        printf " %s", pip[i]
  }
  printf "\n"
}
'
