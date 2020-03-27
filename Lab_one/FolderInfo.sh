#!/bin/bash

# Лабораторная работа №1

package_install()
{
	# Проверяет, установлен ли пакет в системе

	if [ -z "$(dpkg --get-selections | grep "$package")" ] ; then
		echo "Error: Package $package not found. Use 'apt install' command to install it"
		exit 1
	fi
}

path_choice()
{
	# Проверяет введенный пользователем путь до целевого каталога

	if [ "${1:0:1}" = "/" ] || [ "${1:0:1}" = "./" ] ; then
		
		if [ -d "$1" ] ; then
			dpath="$1"
    
		elif [ -f "$1" ] ; then
			echo "Error: Expected directory, not file"
			exit 1

		else
			echo "Error: No such directory"
			exit 1
		fi
	else
		dpath=$("pwd")
	fi
}

head_write()
{
	# Записывает заголовки колонок в таблицу
	
	IFS=:
	echo "" | awk -v arr="${heads[*]}" '{split(arr, heads, ":"); for (head in heads) printf "%s,", heads[head]} END{printf "\n"}' > $output_file
	IFS=$SAVEDIFS
}

size_calc()
{
	# Преобразование размера в байтах в понятный для человека формат
	# Функция создана, так как в некоторых случаях команда du -h [FILE] возвращала некорректное значение
	
	if (( $size < 1024 )) ; then
		return 0
	elif (( $size < $((1024**2)) )) ; then
		size="$(echo "scale=2;$size/1024" |bc)K"
		return 0
	elif (( $size < $((1024**3)) )) ; then
		size="$(echo "scale=2;$size/1024**2" |bc)M"
		return 0
	else
		size="$(echo "scale=2;$size/1024**3" |bc)G"
		return 0
	fi
}

info_write()
{
	# Записывает информацию о файле в таблицу
	
	size="$(stat -c %s $file)"
	size_calc "$size"
	
	if [ -d $file ] ; then
		extension="-"
		files_inside="$(ls "$file" | wc -l)"
		duration="-"
	else
		extension="${file##*.}"
		if [[ "$extension" == "$file" ]] ; then
			extension="-"
		fi

		files_inside="-"
		
		# Извлечение длительности медиафайлов
		duration="$(ffprobe "$file" 2>&1 | awk -F'[:,]' '/Duration/ {printf("%02d:%02d:%02.2f", $2,$3,$4)}')" 
		if [ -z $duration] ; then
			duration="-"
		fi
	fi
		
	file_info=("$number" "$file" "$extension" "$(file -b "$file")" "$size" "$(date -r "$file" +"%T %D")"
			"$(stat -c %U "$file")" "$(stat -c %A "$file")" "$files_inside" "$duration")
	
	IFS=$">"
	echo "" | awk -v arr="${file_info[*]}" '{split(arr,row, ">"); for (col in row) printf "\"%s\",", row[col]} END{printf "\n"}' >> $output_file
	IFS=$SAVEDIFS

	number=$(($number+1))
}

######################
# ОСНОВНАЯ ПРОГРАММА #
######################

output_file="FolderInfo.csv"
package="ffmpeg"
heads=("№" "File Name" "File Extension" "Type of content" "Size" "Date" "Owner" "Access Rights" "Files Inside" "Duration")
number=1

package_install $package
path_choice "$1"
cd $dpath

if [ -z "$(ls)" ] ; then
	echo "Error: Folder is empty"
	exit 0
fi

head_write $heads $output_file
list=(*)

for file in "${list[@]}" ; do
	info_write $file $output_file $number
done

echo "File with information about folder: $dpath/$output_file"
cd - > /dev/null

exit 0
