# Start the process in the background and store its pid
cd agents/bump/
rm -rf runs
rm -rf datasets
setsid bin/concept_bump etc/config &
pid=$!

cleanup() {
	# Kill the full so Python multiprocessing children are not orphaned.
	kill -TERM -- "-$pid" 2>/dev/null || true
	wait "$pid" 2>/dev/null || true
}

# We capture stop signals and terminate the whole process group.
trap cleanup SIGINT SIGTERM EXIT

# Wait for the process to finish
wait $pid
trap - EXIT