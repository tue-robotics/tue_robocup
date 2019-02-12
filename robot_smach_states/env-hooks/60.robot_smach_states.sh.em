robot_console () {
    ROBOT_PARTS_DIR=`rospack find robot_smach_states`/src/robot_smach_states
    ipython -i --no-banner --no-confirm-exit --autocall 2 "${ROBOT_PARTS_DIR}/console.py" -- $*
}

alias amigo-console='robot_console amigo'
alias sergio-console='robot_console sergio'
alias hero-console='robot_console hero'
