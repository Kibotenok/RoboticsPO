# ОПИСАНИЕ РАБОТЫ СКРИПТА ПО ПЕРВОЙ ЛАБОРАТОРНОЙ РАБОТЕ  
**ВЫПОЛНИЛ:**  
Антропов Никита  
Группа R3325  

**ВЫЗОВ СКРИПТА:**  
FolderInfo.sh [DIRECTORY]  

**ОПИСАНИЕ СКРИПТА:**  
Получение информации о файлах и подкаталогах в указанном каталоге в табличном формате (CSV). Полученный файл сохраняется в целевой каталог, путь до файла выводится в консоль. Если при вызове скрипта целевой каталог не указан, используется текущая рабочая директория.

**ОПИСАНИЕ ОШИБОК:**  
| № 	| Сообщение об ошибке                                                      	| Причина                                                                                            	|
|---	|--------------------------------------------------------------------------	|----------------------------------------------------------------------------------------------------	|
| 1 	| Package [PACKAGE] not found. Use 'apt install' command <br>to install it 	| Не обнаружен пакет, необходимый для работы<br>скрипта. Скрипт прекращает работу                    	|
| 2 	| Expected directory, not file                                             	| Параметр [DIRECTORY] получил на вход путь <br>до файла, а не каталога. Скрипт завершает <br>работу 	|
| 3 	| No such directory                                                        	| Указанного в параметре [DIRECTORY] каталога <br>не существует. Скрипт завершает работу             	|
| 4 	| Folder is empty                                                          	| Целевой каталог не содержит каких-либо <br>подкатологов или файлов. Скрипт завершает <br>работу   	|

**ОПИСАНИЕ ПОЛЕЙ ТАБЛИЦЫ:**  
Таблица имеет следующие поля: 
+ № (Порядковый номер);  
+	File Name (Полное имя файла/подкаталога);  
+ File Extension (Расширение файла/подкаталога. При его отсутствии ставится символ '-');  
+ Type of content (Описание контента файла/подкаталога);  
+	Size (Размер файла/подкаталога);  
+	Date (Время и дата последнего изменения файла/подкаталога в формате H:M:S M:D:Y);  
+ Owner (Имя владельца файла/подкаталога);  
+ Access Rights (Права доступа к файлу/подкаталогу);  
+ Files Inside (Количество файлов в подкаталоге. Для файлов ставится символ '-');  
+ Duration (Длительность видео/аудио файлов в формате H:M:S.cs);  

**ПРИМЕР ВЫВОДА:**  
#### Тестовая папка  
![alt text](Screenshots/folder.png "Тестовая папка")
#### Вызов скрипта из терминала и вывод сообщения о завершении его работы  
![alt text](Screenshots/result.png "Вывод сообщения после завершения скрипта")  

**РЕЗУЛЬТАТ:**  
Результат работы скрипта находится в папке Result
