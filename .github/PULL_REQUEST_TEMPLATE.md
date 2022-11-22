#### **Overview**

- \<Overview for request or description for added feature.\>
- \<This section should contain a brief description of the requested resource.\>
- \<Briefly describe the feature. Images can be included to improve the understanding of the functionality.\>

#### **Use the following settings in the overrides file for docker testing**

- Use these settings to compile and run the module using `autoproj`.
- The contents of the files manifest and `overrides.yaml` must be changed according to the `templates` below.
- Additional packages may be needed to test this module.

  e.g.,

- _overrides.yaml_

  ```yaml
    overrides:
      - <package>:
        branch: <branch>
  ```

- _manifest_

  ```yaml
    package_sets:
      - github: Brazilian-Institute-of-Robotics/bir.subot-package_set
        private: true
    layout:
      - <package>
  ```

#### **What was added/changed in this update**

- \<Use the commit messages to describe what was included.\>

#### **Depends On:**

- \<Inform if there is a dependency on other pull requests.\>

#### **Related Issues:**

- \<Inform if the pull request is associated with issues.\>

#### **Notes:**

- Include notes and notes specific to this pull request here, if not, include the expression N/A. (Not Applicable)