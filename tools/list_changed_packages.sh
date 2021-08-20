#!/bin/bash
all_modified_files=$(git diff --name-only origin/master)
all_modified_pkgs=()
for modified_file in ${all_modified_files}; do
	dname=$(dirname ${modified_file})
	while [ "${dname}" != "." ]; do
		if [ -f ${dname}/package.xml ]; then
			pkg_name=$(grep name "${dname}/package.xml")
			all_modified_pkgs+=($(echo ${pkg_name}|cut -d'>' -f2|cut -d'<' -f1))
			break;
		fi
		dname=$(dirname ${dname})
	done
done

for pkg in "${all_modified_pkgs[@]}"; do
	echo $pkg
done
