
const {
  getFooterInfo,
  createCompleteFooterFix
} = require("./footer-utils");

/** @type {import("markdownlint").Rule} */
module.exports = {
  names: ["require-footer"],
  description: "Add both footer tags in one fix when neither exists.",
  information: new URL(
    "https://unl-lunabotics.github.io/docs/Administrative/The-Linter.html#custom-rule-required-footer-tags"
  ),
  tags: ["credit"],
  parser: "none",
  function: (params, onError) => {
    const path = require("path");
    if (path.basename(params.name) === "index.md") return;

    const info = getFooterInfo(params);

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
