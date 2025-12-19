---
title: The Linter
parent: Administrative
nav_order: 3
---

# The Linter

In order to help keep the formatting clear and consistent, this repository utilizes a markdown linter. A linter is a tool that flags any format violations for the writer to fix. For example, the linter we use flags not starting a file with a title as a violation of the formatting guidelines. Please note that it does not catch spelling or grammar mistakes, just formatting. We are using a VSCode extension named markdownlint by David Anson. Documentation for this linter can be found at <https://github.com/DavidAnson/markdownlint/tree/main>

For the most part, this linter should not require much maintenance besides just installing the extension when you first download the repository. It will not work when working on the web version of VSCode, which is one of the reasons why editing in the browser is not recommend

Most of these are pretty basic rules, but we do have some custom settings (discussed below)

<u>Usually, the linter will only lint the current file that you have open, NOT the whole repository.</u> If you would like it to lint the whole repository, open VSCode commands (with either CTRL/CMD + SHIFT + P or just by clicking on the top center bar and adding a >) and run the "Lint all Markdown files in the workspace with markdownlint" command. It should highlight any file containing a violation in red

## Disabled Rules

There are a number of default rules in markdownlint, and we have a few of them disabled

1. Some HTML tags are allowed, notably 's' and 'u'. `<s>` is for strike-through and `<u>` is for underline, which are useful tools for readability in long documents
2. Rule MD034 is the rule that requires every link to be a hyperlink (no bare links), which has been disabled since sometimes it is useful being able to see the whole link
3. Rule MD047 is the rule that requires every page to end with a newline. This has been disabled since the author tag replaces it (explained below)

The list of all default rules can be found here: <https://github.com/DavidAnson/markdownlint/blob/main/doc/Rules.md>

## Custom Rule: Required Author Tag

Maintaining this GitHub Pages is going to be a lot of work for everyone involved. It requires a lot of technical experience to write detailed guides and good information, and it takes a lot of time and effort to acquire that knowledge. It is important to credit every page with its author to acknowledge them

Please note that the author tag is not for the person who last revised the page (GitHub Pages should show revision histories for you), it is for the original author of the information behind the page. Small edits do not give authorship, but large edits should be credited by adding more than one author to the list. It is okay for a page to have many authors, but if at any point the original page's information has been edited so much that the original author should no longer be on there, it is probably the time to make a new page and archive the old one rather than continually updating the same one

<u>One of the most important parts of having an author tag is to allow future communication with the author.</u> If someone wrote a document years ago and is no longer on the team, but you really need to know some more information, it is important to be able to know both 1. Who wrote that page and 2. How to contact them. For the most part, a GitHub account will be sufficient since most people will include their emails on their account. If you do not have your email on your account, please either add it or include additional contact information in your author tag

Author tags will follow the following format and be included at the end of every document. The linter for this repository has been edited to make this a custom rule
`> Author: Firstname Lastname (<github user link>)`

Or, if there are multiple authors,

```md
> Author: Firstname Lastname (<github user link>)
> Author: Firstname Lastname (<github user link>)
> ... however many more authors
```

The order for the author tags does not indicate importance in writing a page, it is usually just in chronological order (though no strict ordering rules will be enforced)

## Custom Rule: Ignore Files Starting with an Underscore

This custom rule makes the linter ignore files that are prepended with an underscore. This is almost entirely here just to make the linter ignore the navigation sidebar, which is named _Sidebar.md. The Sidebar does not need to follow linting rules as it is not a regular page. Additionally, this will make the linter ignore all introduction pages, such as Administrative.md. There is one of these for each subsection and since they're so short they do not require normal linting

Unlike regular custom rules, this one is not implemented via a JavaScript file. This is enforced in two locations that accomplish two different things. Firstly, there is a setting listed in `settings.json` inside the `markdownlint.lintWorkspaceGlobs` that adds `!**/_*.md` as a filter. This prevents files that start with an underscore from being processed by the command "Lint all Markdown files in the workspace with markdownlint". Secondly, in the root of the workspace there is a `.markdownlint-cli2.jsonc` file. All this file does is declare "**/_*.md" as something that should be ignored when the linter runs normally (via saving a file)

## Editing the Linter

The general settings for the linter can be found in `.vscode/settings.json`

Most of the linter configurations will be under the `markdownlint.config` block. This includes disabling specific rules and allowing certain HTML elements. Rule MD033 is the one that disallows any HTML tags, and to add allowed elements to it, you just add "element" into the allowed_elements list shown below

```json
"MD033": {
            "allowed_elements": ["example"]
        }
```

Custom linter rules will be JavaScript files stored inside `.vscode/markdownlint-rules`. Details on how to write a custom rule are located at https://github.com/DavidAnson/markdownlint/blob/main/doc/CustomRules.md. Once the JavaScript is written, you can add it to the custom rules import inside settings.json. You will just add the path to the custom rule inside the `markdownlint.customRules` block (note that this is a top-level block. It does NOT go inside `markdownlint.config`)

```json
"markdownlint.customRules": [
    "./.vscode/markdownlint-rules/custom-rule.js"
  ]
```

> Author: Ella Moody (<https://github.com/TheThingKnownAsKit>)