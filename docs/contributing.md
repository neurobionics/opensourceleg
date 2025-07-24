# Contributing to `opensourceleg`

Contributions are welcome, and they are greatly appreciated!
Every little bit helps, and credit will always be given.

# Types of Contributions

You can contribute in many ways:

## Report Bugs

Report bugs [here](https://github.com/neurobionics/opensourceleg/issues)

If you are reporting a bug, please include:

- Any details about your local setup that might be helpful in troubleshooting.
- Expected behavior vs actual behavior.
- Detailed steps to reproduce the bug.

## Fix Bugs

Look through the GitHub issues for bugs.
Anything tagged with "bug" and "help wanted" is open to whoever wants to implement a fix for it.

## Implement Features

Look through the GitHub issues for features.
Anything tagged with "enhancement" and "help wanted" is open to whoever wants to implement it.

## Write Documentation

The `opensourceleg` package could always use more documentation, whether as part of the official docs, in docstrings, or even on the web in blog posts, articles, and such.

## Submit Feedback

The best way to send feedback is to file an issue at [here](https://github.com/neurobionics/opensourceleg/issues).

If you are proposing a new feature:

- Explain in detail how it would work.
- Keep the scope as narrow as possible, to make it easier to implement.
- Remember that this is a volunteer-driven project, and that contributions
  are welcome :)

# Get Started!

Ready to contribute? Here's how to set up `opensourceleg` for local development.
Please note this documentation assumes you already have `uv` and `Git` installed and ready to go.

1. Fork the `opensourceleg` repo on GitHub.

2. Clone your fork locally:

```bash
cd <directory_in_which_repo_should_be_created>
git clone git@github.com:YOUR_NAME/opensourceleg.git
```

3. Now we need to install the environment. Navigate into the directory

```bash
cd opensourceleg
```

Then, create and activate a virtual environment with uv:

```bash
uv venv
```

Then, activate the virtual environment:

```bash
source .venv/bin/activate
```

Finally, install the dependencies:

```bash
uv sync
```

or, if you want to install all extra/optional dependencies:

```bash
uv sync --all-extras
```

4. Install pre-commit to run linters/formatters at commit time:

```bash
uv run pre-commit install
```

5. Create a branch for local development:

```bash
git checkout -b name-of-your-bugfix-or-feature
```

Now you can make your changes locally.

6. Don't forget to add test cases for your added functionality to the `tests` directory.

7. When you're done making changes, check that your changes pass the formatting tests.

```bash
make check
```

8. Now, validate that all unit tests are passing:

```bash
make test
```

9. Before raising a pull request you should also run tox.
   This will run the tests across different versions of Python:

```bash
tox
```

This requires you to have multiple versions of python installed.
This step is also triggered in the CI/CD pipeline, so you could also choose to skip this step locally.

10. Commit your changes and push your branch to GitHub:

```bash
git add .
git commit -m "Your detailed description of your changes."
git push origin name-of-your-bugfix-or-feature
```

11. Submit a pull request through the GitHub website.

# Commit Message Guidelines

This project uses [Conventional Commits](https://www.conventionalcommits.org/) for commit messages and [Release Please](https://github.com/googleapis/release-please) for automated releases.

## Conventional Commit Format

Commit messages should follow this format:

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

### Types

- `feat`: A new feature
- `fix`: A bug fix
- `docs`: Documentation only changes
- `style`: Changes that do not affect the meaning of the code (white-space, formatting, missing semi-colons, etc)
- `refactor`: A code change that neither fixes a bug nor adds a feature
- `perf`: A code change that improves performance
- `test`: Adding missing tests or correcting existing tests
- `build`: Changes that affect the build system or external dependencies
- `ci`: Changes to CI configuration files and scripts
- `chore`: Other changes that don't modify src or test files
- `revert`: Reverts a previous commit

### Examples

```bash
feat: add support for new actuator model
fix: resolve memory leak in joint controller
docs: update installation instructions
refactor: simplify sensor calibration logic
test: add unit tests for PID controller
```

### Breaking Changes

Breaking changes should be indicated by adding `!` after the type/scope or by including `BREAKING CHANGE:` in the footer:

```bash
feat!: remove deprecated API methods
# or
feat: add new configuration format

BREAKING CHANGE: Configuration file format has changed from JSON to YAML
```

## Release Please

This project uses Release Please to automatically:
- Generate changelogs
- Create releases
- Update version numbers
- Publish to PyPI
- Deploy documentation

Release Please analyzes conventional commit messages to determine the type of release (major, minor, or patch) and generates appropriate release notes. This happens automatically when commits are pushed to the main branch.

### How Release Please Works

1. **On every push to main**: Release Please analyzes new commits using conventional commit format
2. **Creates/updates a Release PR**: If releasable changes are found, it creates or updates a "chore: release X.Y.Z" PR
3. **When Release PR is merged**: A GitHub release is created and the package is automatically published to PyPI

### Avoiding Unintended Releases

If you need to commit to main without triggering a release (for urgent fixes, documentation, etc.), use these strategies:

#### Skip Release Processing
Add `Release-As: skip` to your commit message:

```bash
git commit -m "docs: fix typo in contributing guide

Release-As: skip"
```

#### Use Non-Release Commit Types
Some commit types don't trigger releases by default:
- `chore`: Maintenance tasks
- `ci`: CI/CD changes
- `build`: Build system changes
- `docs`: Documentation-only changes (depending on configuration)

```bash
git commit -m "chore: update GitHub Actions workflow"
```

### Manual Release Control

#### Force a Specific Version
Override the automatic version bump:

```bash
git commit -m "feat: add new actuator support

Release-As: 2.1.0"
```

#### Force a Patch Release
Convert a non-release commit into a patch release:

```bash
git commit -m "docs: improve API documentation

Release-As: patch"
```

### Working with Release Please PRs

When Release Please creates a release PR:

1. **Review the changelog**: Ensure all changes are accurately described
2. **Check the version bump**: Verify it matches the significance of changes
3. **Merge to release**: Merging the PR will create the GitHub release and publish to PyPI
4. **Never close without merging**: This will skip the release entirely

The release PR will also trigger a test publication to Test PyPI for validation.

### Troubleshooting

**No Release PR created?**
- Ensure commits follow conventional commit format exactly
- Check that commits aren't marked with `Release-As: skip`
- Verify there isn't already an open Release Please PR (only one can exist)

**Wrong version bump?**
- `fix:` commits create patch releases (0.0.1)
- `feat:` commits create minor releases (0.1.0)
- Breaking changes (with `!` or `BREAKING CHANGE:`) create major releases (1.0.0)

**Need to cancel a release?**
- Close the Release Please PR (don't merge it)
- The next qualifying commit will create a new release PR

# Pull Request Guidelines

Before you submit a pull request, check that it meets these guidelines:

1. The pull request should include tests.

2. If the pull request adds functionality, the docs should be updated.
   Put your new functionality into a function with a docstring, and add the feature to the list in `README.md`.
