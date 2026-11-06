const {
  getFooterInfo,
  createCompleteFooterFix
} = require("./footer-utils");

/** @type {import("markdownlint").Rule} */
module.exports = {
  names: ["require-footer"],
  description: "Check footer formatting and add both tags when missing.",
  information: new URL(
    "https://unl-lunabotics.github.io/docs/Administrative/The-Linter.html#custom-rule-required-footer-tags"
  ),
  tags: ["credit"],
  parser: "none",
  function: (params, onError) => {
    const path = require("path");
    if (path.basename(params.name) === "index.md") return;

    const info = getFooterInfo(params);

    // Every author line must be followed by a standalone ">" line.
    const authorIndices = info.footerIndices.filter(i =>
      /^>\s*Author:/.test(params.lines[i].trim())
    );

    const missingSpacerIndex = authorIndices.find(i =>
      params.lines[i + 1]?.trim() !== ">"
    );

    if (missingSpacerIndex !== undefined) {
      onError({
        lineNumber: missingSpacerIndex + 1,
        detail: "Missing '>' separator after author footer.",
        context: params.lines[missingSpacerIndex]
      });

      return;
    }

    // The individual rules handle cases where either tag exists.
    if (info.hasAnyTag) return;

    const authorTemplate =
      "> Author: First Last (<https://github.com/your-handle>)";
    const reviewTemplate = "> Last Reviewed: YYYY-MM-DD";

    onError({
      lineNumber: Math.max(info.end + 1, 1),
      detail: "Missing author and last-reviewed footer tags.",
      context: params.lines[Math.max(info.end, 0)] || "",
      fixInfo: createCompleteFooterFix(
        params,
        authorTemplate,
        reviewTemplate
      )
    });
  }
};
