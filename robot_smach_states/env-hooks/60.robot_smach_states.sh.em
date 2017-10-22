robot_console () {
    depcheck ipython
    ROBOT_PARTS_DIR=`rospack find robot_smach_states`/src/robot_smach_states
    ipython -i --no-banner --no-confirm-exit --autocall 2 "${ROBOT_PARTS_DIR}/console.py" -- $*
}

# bash completion

# _robot_console()
# {
#     local cur=${COMP_WORDS[COMP_CWORD]}
#     COMPREPLY=( $(compgen -W "arms head ears base perception speech torso" -- $cur) )
# }
# complete -F _robot_console robot_console

alias amigo-console='robot_console amigo'
alias sergio-console='robot_console sergio'
