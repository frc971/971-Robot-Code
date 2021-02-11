_aosdump_completions() {
  eval "$($COMP_LINE --_bash_autocomplete --_bash_autocomplete_word=${COMP_WORDS[COMP_CWORD]})"
  return 0;
}

complete -F _aosdump_completions -o default aos_dump
complete -F _aosdump_completions -o default aos_dump.stripped
complete -F _aosdump_completions -o default aos_send
complete -F _aosdump_completions -o default aos_send.stripped
