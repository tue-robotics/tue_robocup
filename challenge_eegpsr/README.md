# Features

## All

Actions:
* Move to
* Find object
* Grasp object
* Move object to
* Find person
* Answer question

Misc:
* Speech recognition
* Parse sentence to semantics
* Resolve 'it'
* Resolve 'me'
* Confirmation
* Support up to three actions in one command

## GPSR + EEGPSR only

Introduction:
* Robot needs to enter through the door?

Misc:
* Integrate QR recognition

## Open + Final Only

## GPSR Only

## EEGPSR Only

Actions:
* Follow person
* Find waving person

Misc:
* Fill in missing semantics by asking questions
* Remember object positions

# Architecture

Functions:

    def set_grammar(grammar_file, knowledge)

    def request_command(ask_confirmation=True, ask_missing_info=False)
        returns: None if getting command failed, otherwise (command_sentence, command_semantics)

    def parse_command(command_sentence)
        returns: command_semantics

    def execute_command(command_semantics, blocking=True)

    def wait_for_command()

Steps:
 * Get sentence from speech recognition
 * Parse sentence to semantics
 * Resolve 'it' and 'me' in semantics
(* Fill in missing semantics by asking questions)
 * Iterate over the actions in the semantics, and execute them
