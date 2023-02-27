Javascript-related Information
================================================================================

Debugging Cypress tests
--------------------------------------------------------------------------------
You can run Cypress tests interactively. I.e. the browser will open up and you
can interact with it. Use the `tools/js/run_cypress_test_interactively.sh`
script for this.

```console
$ ./tools/js/run_cypress_test_interactively.sh //path/to:test
(opens Chrome and runs tests)
```

All arguments to the script are passed to `bazel test`. So you can specify `-c
opt` and other bazel options.

### Pausing execution
Use [`cy.pause()`](https://docs.cypress.io/api/commands/pause) to pause
execution. Resume by hitting the green "Play" icon near the top left.

Pausing can be helpful when you're trying to understand more about the state of
the web page in the middle of a test.

### Getting `console.log()` output
Add `--test_env=DEBUG=cypress:launcher:browsers` to your test execution.
Cypress will then log Chrome's output. Among which will be all `console.log()`
statements.
