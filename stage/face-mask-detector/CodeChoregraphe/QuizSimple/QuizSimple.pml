<?xml version="1.0" encoding="UTF-8" ?>
<Package name="QuizSimple" format_version="4">
    <Manifest src="manifest.xml" />
    <BehaviorDescriptions>
        <BehaviorDescription name="behavior" src="behavior_1" xar="behavior.xar" />
        <BehaviorDescription name="behavior" src="behavior_quiz" xar="behavior.xar" />
    </BehaviorDescriptions>
    <Dialogs>
        <Dialog name="quizSimple" src="quizSimple/quizSimple.dlg" />
        <Dialog name="quizMemoire" src="quizMemoire/quizMemoire.dlg" />
    </Dialogs>
    <Resources />
    <Topics>
        <Topic name="quizSimple_frf" src="quizSimple/quizSimple_frf.top" topicName="quizSimple" language="fr_FR" />
        <Topic name="quizMemoire_frf" src="quizMemoire/quizMemoire_frf.top" topicName="quizMemoire" language="fr_FR" />
    </Topics>
    <IgnoredPaths />
    <Translations auto-fill="en_US">
        <Translation name="translation_en_US" src="translations/translation_en_US.ts" language="en_US" />
        <Translation name="translation_fr_FR" src="translations/translation_fr_FR.ts" language="fr_FR" />
    </Translations>
</Package>
