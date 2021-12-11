gnome-terminal -x sh -c  "roscore; bash"
cd ../../devel/lib/allstar/
valgrind --leak-check=full --log-file=../../../src/allstar/results/valgrind_results.txt ./main 4 ../../../src/allstar/src/test4.jpg
echo "Done Processing. Results are stored in results/valgrind_results.txt"