docker compose exec -T racecar bash << 'LOCAL'
    echo "Now inside $PWD"

    echo "racecar@mit" | sudo -S apt-get install sshpass -y
    sshpass -p "racecar@mit" ssh -tt racecar << 'REMOTE'
        echo "Now inside $PWD"
        cd
        # Using script to force a pseudo-TTY for sudo
        echo "racecar@mit" | script -q -c "sudo -S ./run_rostorch.sh" /dev/null
    REMOTE
LOCAL
