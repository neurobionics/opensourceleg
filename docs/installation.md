
### Developing and Contributing to the Library

If you'd like to modify or contribute to the [opensourceleg](https://pypi.org/project/opensourceleg/) library, we recommend following these steps:

1. **Fork the repository** by clicking the "Fork" button at the top right of this [page](https://github.com/neurobionics/opensourceleg).

2. **Clone your fork** to your local machine:

   ```bash
   git clone https://github.com/YOUR-USERNAME/opensourceleg.git
   cd opensourceleg
   ```

3. **Set up the upstream remote** to keep your fork in sync with the main repository:

   ```bash
   git remote add upstream https://github.com/neurobionics/opensourceleg.git
   ```

4. **Install Poetry** if you haven't already. [Poetry](https://python-poetry.org) is a python packaging and dependency management tool. We use poetry to manage dependencies and build the library.

5. **Install dependencies and activate the virtual environment**:

   ```bash
   poetry install
   poetry shell
   ```

6. **Install pre-commit hooks**:

   ```bash
   make install
   ```

7. **Create a new branch** for your feature or bugfix:

   ```bash
   git checkout -b feature-or-bugfix-name
   ```

8. **Make your changes** and commit them with descriptive messages.

9. **Run checks**. See [the contributing guidelines](https://github.com/neurobionics/opensourceleg/blob/main/CONTRIBUTING.md) for more information.

10. **Push your changes** to your fork:

```bash
git push origin feature-or-bugfix-name
```

11. **Create a Pull Request** by navigating to your fork on GitHub and clicking `New Pull Request`.

Your changes will be reviewed by the maintainers, and if approved, they will be merged into the main repository.