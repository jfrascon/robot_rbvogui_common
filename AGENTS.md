# Repository Instructions for Agents

## Conventional Commits

All commit messages MUST follow the Conventional Commits specification.

This rule only governs the commit message text. The user decides which changes belong in each commit.

Use one of the following types, intentionally aligned with the repository's `conventional-pre-commit` validation:

`build`, `chore`, `ci`, `docs`, `feat`, `fix`, `perf`, `refactor`, `revert`, `style`, or `test`.

- Use the format `type(scope): description`, where the scope is optional.
- Use a lowercase type.
- Write the description as a concise imperative statement without a trailing period.
- Mark breaking changes with `!` before the colon or with a `BREAKING CHANGE:` footer.
- If `!` is used, the description should describe the breaking change; a `BREAKING CHANGE:` footer may still be added for extra detail.
- For non-trivial commits, add a concise body after a blank line explaining what changed and why it matters.
- Mention relevant behavior, configuration, dependency, launch, model, or interface changes when useful.
- Do not repeat the diff or write an exhaustive changelog.
- Omit the body for trivial commits where the description is sufficient.
- Keep system-generated merge commit messages as generated.
- Do not introduce additional commit types unless explicitly requested by the user or deliberately added to the repository policy.
