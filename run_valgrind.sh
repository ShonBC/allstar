cd build 
valgrind --leak-check=full --log-file=../results/valgrind_results.txt ./app/shell-app
echo "Done Processing. Results are stored in results/valgrind_results.txt"