T=1800 # time to record (each recording) [s]
N=16 # number of recordings

j=1
for (( i=1; i<=$N; i++ ))
do
    fn="recording"${j}.bag
    echo Recording to $fn \($i\)
    ./rs-record -t $T -f $fn
    if [ $? -eq 2 ]; then # EXIT_NAN = 2
        echo "has NaN"
        ((j++))
    else
        echo "no NaN" # => overwrite
    fi
done
