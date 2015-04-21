#!/usr/bin/env python




def replace_word(string,word_in,word_out):
    try:
        if string[:(len(word_in)+1)] == (word_in+" "):
            string = string.replace(string[:len(word_in)],word_out)

        if string[(len(string)-len(word_in)-1):] == (" "+word_in):
            string = string.replace(string[(len(string)-len(word_in)):],word_out)

        string = string.replace(" "+word_in+" "," "+word_out+" ")

    except KeyError:
        print "[gpsr] Received action is to short."

    return string

result = "tell the time"

say_result = replace_word(result,"me","you")

print "say_result = ", say_result

result = "tell me"

say_result = replace_word(result,"me","you")

print "say_result = ", say_result

result = "meet me in the corner of me time"

say_result = replace_word(result,"me","you")

print "say_result = ", say_result