---
title: The Linter
parent: Administrative
nav_order: 3
---

# The Linter
{: .no_toc }

In order to help keep the formatting clear and consistent, this repository utilizes a markdown linter. A linter is a tool that flags any format violations for the writer to fix. For example, the linter we use flags not starting a file with a title as a violation of the formatting guidelines. Please note that it does not catch spelling or grammar mistakes, just formatting. We are using a VSCode extension named markdownlint by David Anson. Documentation for this linter can be found at <https://github.com/DavidAnson/markdownlint/tree/main>.

For the most part, this linter should not require much maintenance besides just installing the extension when you first download the repository. It will not work when working on the web version of VSCode, which is one of the reasons why editing in the browser is not recommend.

Most of these are pretty basic rules, but we do have some custom settings (discussed below):

**Usually, the linter will only lint the current file that you have open, NOT the whole repository.** If you would like it to lint the whole repository, open VSCode commands (with either CTRL/CMD + SHIFT + P or just by clicking on the top center bar and adding a `>`) and run the "Lint all Markdown files in the workspace with markdownlint" command. It should highlight any file containing a violation in red.

## Table of Contents
{: .no_toc .text-delta }

- Table of Contents
{:toc}

## Disabled Rules

There are a number of default rules in markdownlint, and we have a few of them disabled:

1. Some HTML tags are allowed, notably 's' and 'u'. `<s>` is for strike-through and `<u>` is for underline, which are useful tools for readability in long documents. `<mark>` (highlighting) and `<details>`/`<summary>` (collapsible sections) are also allowed.
2. Rule MD034 is the rule that requires every link to be a hyperlink (no bare links), which has been disabled since sometimes it is useful being able to see the whole link.
3. Rule MD047 is the rule that requires every page to end with a newline. This has been disabled since the author tag replaces it (explained below).
4. Rule MD013 is the rule that limits line length. This has been disabled since long lines are common in prose and links.
5. Rule MD025 is the rule that requires only one top-level heading per page. This has been disabled since the page title already comes from the front matter.
6. Rule MD059 is the rule that requires descriptive link text. This has been disabled since short link labels are sometimes clearer in context.

The list of all default rules can be found here:  
<https://github.com/DavidAnson/markdownlint/blob/main/doc/Rules.md>

## Custom Rules

Maintaining this repository is a lot of work for everyone involved. It requires a lot of technical experience to write detailed guides and good information. It takes a lot of time and effort to acquire that knowledge. Because of this, we have a couple of custom rules to ensure everyone gets credit and that it is up-to-date.

### Required Author Tag

It is important to credit every page with its author to acknowledge them.

Please note that the author tag is not for the person who last revised the page (git history does this already). It is for the original author of the information behind the page. Small edits do not give authorship, but large edits should be credited by adding more than one author to the list. It is okay for a page to have many authors, but if at any point the original page's information has been edited so much that the original author should no longer be on there, it is probably time to make a new page and archive the old one rather than continually updating the same one.

**One of the most important parts of having an author tag is to allow future communication with the author.** If someone wrote a document years ago and is no longer on the team, but you really need to know some more information, it is important to be able to know both who wrote that page and how to contact them. For the most part, a GitHub account will be sufficient since most people will include their emails on their account. If you do not have your email on your account, please either add it or include additional contact information in your author tag.

Author tags will follow the following format and be included at the end of every document. The linter for this repository has been edited to make this a custom rule.

```md
> Author: Firstname Lastname (<github user link>)
```

Or, if there are multiple authors,

```md
> Author: Firstname Lastname (<github user link>)
> Author: Firstname Lastname (<github user link>)
> ... however many more authors
```

The order for the author tags does not indicate importance in writing a page, it is usually just in chronological order (though no strict ordering rules will be enforced)

### Last Reviewed Tag

In addition to the author tag, we also have a last reviewed tag. The purpose of this is to ensure documentation is up-to-date. Last reviewed tags will follow the following format and be included **after** the author tag. The linter for this repository has been edited to make this a custom rule.

```md
> Last Reviewed: YYYY-MM-DD
```

After editing a document, this tag should be updated.

Finally, the linter enforces documentation to be reviewed after 6 months. This ensures documentation is still accurate and up-to-date months later.

Once the linter flags documentation for review, anyone can read and update it. If during review, it is found that a documentation is no longer applicable, it is recommended to put the out-of-date file(s) in the Archive, then make a new file. For minor revisions, it may instead be sufficient to update small parts of the existing file.

## Editing the Linter

Most of the linter configurations will in the `.markdownlint.json` file. This includes disabling specific rules and allowing certain HTML elements. Rule MD033 is the one that disallows any HTML tags, and to add allowed elements to it, you just add "element" into the allowed_elements list shown below.

```json
"MD033": {
  "allowed_elements": ["example"]
}
```

Custom linter rules are JavaScript files stored inside `.vscode/markdownlint-rules`. Details on how to write a custom rule are located at <https://github.com/DavidAnson/markdownlint/blob/main/doc/CustomRules.md>. Once the JavaScript is written, you can add it to the custom rules import inside `.markdownlint-cli2.jsonc`. You will just add the path to the custom rule inside the `customRules` block.

```json
"customRules": [
  "./.vscode/./markdownlint-rules/custom-rule.js",
]
```

## HTML-Proofer

[HTML-Proofer](https://github.com/gjtorikian/html-proofer) is a testing suite designed to validate HTML output. We primarily use it here to check for dead/nonfunctional links in the documentation. It has to be run from the command line, and it scans the built HTML code for errors.

`bundle exec jekyll build` compiles the written markdown files into a full HTML website inside the `_site/` folder. This is the folder HTML-Proofer checks, so the build command needs to be run first.

To check every single link in the project, run:

```bash
bundle exec jekyll build && bundle exec htmlproofer ./_site --ignore-status-codes "403,429"
```

{: .note }
The `--ignore-status-codes` flag hides failures that return the listed error codes. `403` means a website refused the request, usually because it blocks bots. `429` means the website is rate-limiting requests, which is common because HTML-Proofer checks many links quickly. Neither usually means the link is dead, so they are ignored.

If we want to only check links that point to other parts of the documentation (i.e. filtering out external websites), we use the `--disable-external` flag:

```bash
bundle exec jekyll build && bundle exec htmlproofer ./_site --disable-external
```

You might want to run this since it is generally more reliable than the full scan, as the full scan will sometimes produce false error flags (more info on how to handle these below).

### Reading the Output

Each failure lists the file, line, and reason, grouped by the type of check that found it. For example:

```text
For the Images check, the following failures were found:

* At ./_site/docs/Archive/Gazebo-URDF.html:138:

  internal image ../../../../../attachments/ros_vs_gazebo_coords.png does not exist

For the Links > Internal check, the following failures were found:

* At ./_site/docs/Administrative/Docs-Reference.html:61:

  internally linking to /docs/Administrative/Editing-the-Docs.html#Testing Locally; the file exists, but the hash 'TestingLocally' does not

For the Links > External check, the following failures were found:

* At ./_site/docs/Embedded Systems/Technical/RealSense.html:1:

  External link https://github.com/cscheet2 failed (status code 404)
```

The path after `At` is the generated HTML file, which matches the source file in `docs/` (for example, `_site/docs/Archive/Gazebo-URDF.html` comes from `docs/Archive/Gazebo-URDF.md`). The line number refers to the HTML, not the Markdown, so use the link or image from the message to find it in the source with `grep`:

```bash
grep -rn "ros_vs_gazebo_coords" docs/
```

Here is an explanation of the errors presented in the example output:

- **Images:** the image path points to a file that doesn't exist.
- **Links > Internal:** the page exists, but the `#anchor` doesn't. Heading IDs are lowercase with hyphens, so `#Testing Locally` should be `#testing-locally`.
- **Links > External:** a website returned an error. A `404` means the page is gone, but other codes may be false positives (see below).

If a link fails but works in a browser, it is likely a false positive from bot protection. Open the link to confirm, then skip it with `--ignore-urls`, which takes a comma-separated list of regular expressions:

```bash
bundle exec htmlproofer ./_site --ignore-urls "/example\.com/,/another\.com/"
```

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)  
> Author: Jesse Mills (<https://github.com/JesseMills0>)
> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
