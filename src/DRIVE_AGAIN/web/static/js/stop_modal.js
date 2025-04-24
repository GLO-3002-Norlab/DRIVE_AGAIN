document.getElementById("reason-select").addEventListener("change", function (event) {
    let value = document.getElementById("reason-select").value;

    document.getElementById("stop-reason-submit").disabled = value === "";

    let otherReasonInput = document.getElementById("other-reason-input");
    if (value === "other") {
        otherReasonInput.style.display = "flex";
        otherReasonInput.required = true;
    } else {
        otherReasonInput.style.display = "none";
        otherReasonInput.required = false;
    }
});

document.getElementById("reason-form").addEventListener("submit", function () {
    if (document.getElementById("reason-select").value !== "other") {
        closeStopModal(document.getElementById("reason-select").value);
    } else {
        closeStopModal(document.getElementById("other-reason-input").value);
    }
});