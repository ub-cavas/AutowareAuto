#!/bin/bash
all_pkgs=`colcon list`
all_modified_files=`git diff --name-only origin/master`
all_modified_pkgs=()
while IFS= read -r file; do
	echo $file
	echo
	while IFS= read -r pkg; do
		pkg_path=$(echo ${pkg} | awk '{print $2}')
		match=`echo $file | grep $pkg_path`
		if [ ! -z ${match} ]; then
			pkg_name=$(echo ${pkg} | awk '{print $1}')
			all_modified_pkgs+=( $pkg_name )
		fi
	done < <(printf '%s\n' "$all_pkgs")
done < <(printf '%s\n' "$all_modified_files")
